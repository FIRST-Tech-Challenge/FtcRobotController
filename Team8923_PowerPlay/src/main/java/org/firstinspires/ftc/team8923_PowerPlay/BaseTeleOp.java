package org.firstinspires.ftc.team8923_PowerPlay;

import static org.firstinspires.ftc.team8923_PowerPlay.Constants.CLOSED_CLAW;
import static org.firstinspires.ftc.team8923_PowerPlay.Constants.OPEN_CLAW;

import com.qualcomm.robotcore.util.Range;

abstract public class BaseTeleOp extends BaseOpMode {

    private boolean isSlowMode = false;

    private Toggle driveSpeedToggle = new Toggle();

    double mechanismSpeed = 0.9;

    public void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotationalPower = gamepad1.right_stick_x;

        double angle = Math.toDegrees(Math.atan2(y, x)); // 0 degrees is forward
        double power = calculateDistance(x, y);

        driveMecanum(angle, power, rotationalPower);
    }

    public void driveRobotSpeed() {
        isSlowMode = driveSpeedToggle.toggle(gamepad1.left_bumper);
        if (isSlowMode) {
            driveSpeed = 0.50;
        } else {
            driveSpeed = 1.0;
        }
    }

    /**
     * if dpad up, linear slides go up
     * if dpad down, linear slides go down
     */
    public void driveMechanism() {
        if (gamepad2.dpad_up) {
            motorSlideLeft.setPower(mechanismSpeed);
            motorSlideRight.setPower(mechanismSpeed);
        } else if (gamepad2.dpad_down
                && motorSlideLeft.getCurrentPosition() > bottomMotorSlideLeft
                && motorSlideRight.getCurrentPosition() > bottomMotorSlideRight) {
            motorSlideLeft.setPower(-mechanismSpeed);
            motorSlideRight.setPower(-mechanismSpeed);
        } else {
            motorSlideLeft.setPower(0);
            motorSlideRight.setPower(0);
        }
    }

    /**
     * press the X button, claw opens
     * press the A button, claw closes
     */
    public void driveClaw() {
        if (gamepad2.x) {
            servoClaw.setPosition(OPEN_CLAW);
        } else if (gamepad2.a) {
            servoClaw.setPosition(CLOSED_CLAW);
        }
    }
}


