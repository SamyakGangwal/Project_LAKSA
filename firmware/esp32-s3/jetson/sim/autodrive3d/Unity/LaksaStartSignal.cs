// LAKSA start signal: the red arm shows at the start; on "go" the pinwheel turns
// 90 degrees so the green arm takes its place (as the timekeeper does on the day).
// Press G in the simulator to start the turn, R to reset to red; or set
// autoStartAfterSeconds > 0 for an automatic start.
using UnityEngine;

public class LaksaStartSignal : MonoBehaviour
{
    public Transform pinwheel;
    public float turnSeconds = 0.6f;           // how long the swing takes by hand
    public float autoStartAfterSeconds = 0f;   // 0 = wait for G
    float startAngle, t = -1f;

    void Start()
    {
        startAngle = pinwheel != null ? pinwheel.localEulerAngles.z : 0f;
        if (autoStartAfterSeconds > 0f) Invoke(nameof(Go), autoStartAfterSeconds);
    }

    public void Go() { if (t < 0f) t = 0f; }

    public void ResetToRed()
    {
        t = -1f;
        if (pinwheel != null) pinwheel.localRotation = Quaternion.Euler(0, 0, startAngle);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.G)) Go();
        if (Input.GetKeyDown(KeyCode.R)) ResetToRed();
        if (t < 0f || pinwheel == null) return;
        t = Mathf.Min(t + Time.deltaTime, turnSeconds);
        pinwheel.localRotation = Quaternion.Euler(0, 0, startAngle - 90f * (t / turnSeconds));
    }
}
