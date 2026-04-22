from algorithms.pid import PIDConfig, PIDController


def test_pid_clamps_output_to_limit():
    controller = PIDController(
        PIDConfig(
            kp=1.0,
            ki=0.0,
            kd=0.0,
            output_limit=0.5,
            integral_limit=10.0,
            deadband=0.0,
            derivative_alpha=0.5,
        )
    )

    output = controller.update(error=10.0, now_s=1.0)
    assert output == 0.5


def test_pid_deadband_zeroes_small_error():
    controller = PIDController(
        PIDConfig(
            kp=1.0,
            ki=0.0,
            kd=0.0,
            output_limit=1.0,
            integral_limit=10.0,
            deadband=0.2,
            derivative_alpha=0.5,
        )
    )

    output = controller.update(error=0.1, now_s=1.0)
    assert output == 0.0


def test_pid_reset_clears_integral_and_history():
    controller = PIDController(
        PIDConfig(
            kp=0.1,
            ki=1.0,
            kd=0.0,
            output_limit=10.0,
            integral_limit=10.0,
            deadband=0.0,
            derivative_alpha=0.5,
        )
    )

    controller.update(error=2.0, now_s=1.0)
    controller.update(error=2.0, now_s=2.0)
    controller.reset()
    output = controller.update(error=2.0, now_s=3.0)
    assert output == 0.2
