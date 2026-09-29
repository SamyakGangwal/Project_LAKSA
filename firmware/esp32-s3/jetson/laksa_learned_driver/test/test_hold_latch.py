import unittest

from laksa_learned_driver.hold_latch import HoldLatch

TIMEOUT = 0.3


class Page:
    """Drives HoldLatch the way console_node does, tracking the heartbeat."""

    def __init__(self) -> None:
        self.latch = HoldLatch()
        self.last_heartbeat = 0.0

    def press(self, now: float) -> bool:
        accepted = self.latch.press(now, self.last_heartbeat, TIMEOUT)
        if accepted:
            self.last_heartbeat = now
        return accepted

    def release(self) -> None:
        self.latch.release()
        self.last_heartbeat = 0.0

    def stop(self) -> None:
        self.latch.stop()
        self.last_heartbeat = 0.0

    def tick(self, now: float) -> bool:
        return self.latch.lapse(now, self.last_heartbeat, TIMEOUT)


class HoldLatchTest(unittest.TestCase):
    def test_continuous_hold_is_accepted(self):
        page = Page()
        for i in range(20):
            self.assertTrue(page.press(10.0 + 0.05 * i))
        self.assertFalse(page.latch.awaiting_release)

    def test_hold_resuming_after_heartbeat_gap_is_ignored_until_release(self):
        page = Page()
        self.assertTrue(page.press(10.0))
        self.assertTrue(page.press(10.05))
        # Wi-Fi drop: next heartbeat 1 s later while the button is still held.
        self.assertFalse(page.press(11.05))
        self.assertFalse(page.press(11.10))
        self.assertTrue(page.latch.awaiting_release)
        page.release()
        self.assertTrue(page.press(12.0))
        self.assertFalse(page.latch.awaiting_release)

    def test_timer_detects_the_loss_before_the_next_press(self):
        page = Page()
        page.press(10.0)
        self.assertFalse(page.tick(10.2))
        self.assertTrue(page.tick(10.4))          # detected once
        self.assertFalse(page.tick(10.5))
        self.assertFalse(page.press(10.6))

    def test_go_is_treated_like_hold(self):
        page = Page()
        page.press(10.0)                          # "go" uses the same latch
        self.assertFalse(page.press(20.0))
        page.release()
        self.assertTrue(page.press(21.0))

    def test_deliberate_release_then_press_is_unchanged(self):
        page = Page()
        page.press(10.0)
        page.release()
        self.assertTrue(page.press(10.1))
        self.assertTrue(page.press(10.15))

    def test_stop_then_press_is_unchanged(self):
        page = Page()
        page.press(10.0)
        page.stop()
        self.assertFalse(page.tick(15.0))
        self.assertTrue(page.press(15.0))

    def test_socket_close_mid_hold_needs_release(self):
        page = Page()
        page.press(10.0)
        page.last_heartbeat = 0.0                 # console_node zeroes the heartbeat when a socket closes
        self.assertTrue(page.tick(10.05))
        self.assertFalse(page.press(10.1))
        page.release()
        self.assertTrue(page.press(10.2))

    def test_first_press_after_idle_is_accepted(self):
        page = Page()
        self.assertFalse(page.tick(100.0))
        self.assertTrue(page.press(100.0))


if __name__ == "__main__":
    unittest.main()
