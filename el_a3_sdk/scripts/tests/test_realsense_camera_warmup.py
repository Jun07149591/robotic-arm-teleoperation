import unittest

from el_a3_sdk.realsense.camera import RealSenseD435


class _FakePipeline:
    def __init__(self, failures_before_success: int) -> None:
        self.failures_before_success = failures_before_success
        self.wait_calls = 0

    def wait_for_frames(self, timeout_ms: int):
        self.wait_calls += 1
        if self.wait_calls <= self.failures_before_success:
            raise RuntimeError("Frame didn't arrive within 5000")
        return object()


def _camera_with_pipeline(pipeline: _FakePipeline) -> RealSenseD435:
    camera = object.__new__(RealSenseD435)
    camera._profile = object()
    camera._pipeline = pipeline
    return camera


class RealSenseCameraWarmupTest(unittest.TestCase):
    def test_warmup_retries_transient_frame_timeouts(self) -> None:
        pipeline = _FakePipeline(failures_before_success=2)
        camera = _camera_with_pipeline(pipeline)

        camera.warmup(
            frame_count=3,
            timeout_ms=5000,
            startup_retry_count=2,
            retry_delay_s=0,
        )

        self.assertEqual(pipeline.wait_calls, 5)

    def test_warmup_reports_timeout_after_retry_budget_is_exhausted(self) -> None:
        pipeline = _FakePipeline(failures_before_success=3)
        camera = _camera_with_pipeline(pipeline)

        with self.assertRaisesRegex(RuntimeError, "RealSense warmup failed"):
            camera.warmup(
                frame_count=1,
                timeout_ms=5000,
                startup_retry_count=2,
                retry_delay_s=0,
            )

        self.assertEqual(pipeline.wait_calls, 3)


if __name__ == "__main__":
    unittest.main()
