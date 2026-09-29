#!/usr/bin/env bash
# Start-reliability loop: run setup/traction_bench.py N times at one speed and
# tabulate each start.  WHEELS OFF THE GROUND, drive_supervisor stopped, a
# person watching the car.
#
#   start_reliability.sh N [speed_mps=0.22] [gap_s=3]
#
# Each run is the bench's --steps ladder: brake 1 s, <speed> 3 s, brake 2 s.
# A bench abort or refusal counts as a failed start and the loop continues.
# The loop stops (without running the bench) if anything else publishes
# /laksa/command or /laksa/brake.
set -uo pipefail

N="${1:-}"
SPEED="${2:-0.22}"
GAP="${3:-3}"
if [[ ! "${N}" =~ ^[1-9][0-9]*$ || ! "${SPEED}" =~ ^-?[0-9]*\.?[0-9]+$ || ! "${GAP}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    echo "usage: $0 N [speed_mps=0.22] [gap_s=3]" >&2; exit 64
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH="${HERE}/../traction_bench.py"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
[[ -f "${HOME}/zed_ws/install/setup.bash" ]] && source "${HOME}/zed_ws/install/setup.bash"
source "${HOME}/third_party/third_party_ws/install/setup.bash"
source "${HOME}/laksa_ws/install/setup.bash"
set -u

OUT="${HOME}/laksa_run/start_reliability_$(date +%Y%m%dT%H%M%S)"
mkdir -p "${OUT}"
echo "start reliability: ${N} runs at ${SPEED} m/s, ${GAP} s apart -> ${OUT}"

for i in $(seq 1 "${N}"); do
    pubs="$(python3 "${HERE}/topic_publishers.py" /laksa/command /laksa/brake)"
    if echo "${pubs}" | awk '$2 != 0 {bad=1} END {exit !bad}'; then
        echo "STOPPING LOOP before run ${i}: another command publisher is present:" >&2
        echo "${pubs}" | sed 's/^/  /' >&2
        echo "${i} other-publisher" > "${OUT}/stopped"
        break
    fi
    echo "--- run ${i}/${N}"
    python3 "${BENCH}" --steps "${SPEED}" --csv "${OUT}/run_${i}.csv" > "${OUT}/run_${i}.log" 2>&1
    echo "$?" > "${OUT}/run_${i}.rc"
    grep -E "REFUSING|ABORT|first rotation|spin-down" "${OUT}/run_${i}.log" | sed 's/^/   /'
    [[ ${i} -lt ${N} ]] && sleep "${GAP}"
done

python3 - "${OUT}" "${SPEED}" <<'PY'
import csv, glob, os, re, sys
out, speed = sys.argv[1], float(sys.argv[2])
STEADY_S = 1.0          # mean of the last second of the 3 s step
rows = []
runs = sorted(glob.glob(os.path.join(out, "run_*.rc")), key=lambda p: int(re.findall(r"run_(\d+)", p)[0]))
for rc_path in runs:
    i = int(re.findall(r"run_(\d+)", rc_path)[0])
    rc = int(open(rc_path).read().strip() or 99)
    log = open(os.path.join(out, f"run_{i}.log"), errors="replace").read()
    note = ""
    m = re.search(r"(REFUSING TO START: .*|ABORT: .*)", log)
    if m:
        note = m.group(1)[:70]
    samples = []
    csv_path = os.path.join(out, f"run_{i}.csv")
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            samples = [r for r in csv.DictReader(f) if r["step"] == "1"]
    spin = peak = steady = None
    if samples:
        t0 = float(samples[0]["t"])
        for r in samples:
            if abs(float(r["measured_erpm"])) > 100:
                spin = float(r["t"]) - t0
                break
        peak = max(abs(float(r["motor_a"])) for r in samples)
        t_end = float(samples[-1]["t"])
        tail = [float(r["measured_erpm"]) for r in samples if float(r["t"]) >= t_end - STEADY_S]
        steady = sum(tail) / len(tail) if tail else None
    ok = rc == 0 and spin is not None
    rows.append((i, rc, ok, spin, peak, steady, note))

fmt = lambda v, f: "-" if v is None else f.format(v)
print(f"\nstart reliability at {speed:+.2f} m/s  ({out})")
print(f"{'run':>3}  {'exit':>4}  {'start':>5}  {'t_spin_s':>8}  {'peak_A':>6}  {'steady_eRPM':>11}  note")
for i, rc, ok, spin, peak, steady, note in rows:
    print(f"{i:>3}  {rc:>4}  {'OK' if ok else 'FAIL':>5}  {fmt(spin, '{:.2f}'):>8}  {fmt(peak, '{:.1f}'):>6}  "
          f"{fmt(steady, '{:.0f}'):>11}  {note}")
good = [r for r in rows if r[2]]
spins = [r[3] for r in good]
print(f"\n{len(good)}/{len(rows)} clean starts" + (f"; time to spin min/mean/max "
      f"{min(spins):.2f}/{sum(spins)/len(spins):.2f}/{max(spins):.2f} s" if spins else ""))
PY
