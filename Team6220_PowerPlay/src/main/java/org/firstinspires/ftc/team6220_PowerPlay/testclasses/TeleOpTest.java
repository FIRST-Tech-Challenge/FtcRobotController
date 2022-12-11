package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Disabled
@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends BaseTeleOp {

    @Override
    public void runOpMode() {
        double p = 10.0, i = 3.0, d = 0.0, f = 0.0;

        initialize();
        waitForStart();

        // get angle after startup to prevent jitter on startup
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive()) {
            driveChassisWithController();

            if (gamepad1.a) {
                if (gamepad1.dpad_up) {
                    p++;
                } else if (gamepad1.dpad_down) {
                    p--;
                }

            } else if (gamepad1.x) {
                if (gamepad1.dpad_up) {
                    i++;
                } else if (gamepad1.dpad_down) {
                    i--;
                }

            } else if (gamepad1.y) {
                if (gamepad1.dpad_up) {
                    d++;
                } else if (gamepad1.dpad_down) {
                    d--;
                }

            } else if (gamepad1.b) {
                if (gamepad1.dpad_up) {
                    f++;
                } else if (gamepad1.dpad_down) {
                    f--;
                }
            }

            motorFL.setVelocityPIDFCoefficients(p, i, d, f);
            motorFR.setVelocityPIDFCoefficients(p, i, d, f);
            motorBL.setVelocityPIDFCoefficients(p, i, d, f);
            motorBR.setVelocityPIDFCoefficients(p, i, d, f);

            telemetry.addData("p", p);
            telemetry.addData("i", i);
            telemetry.addData("d", d);
            telemetry.addData("f", f);
            telemetry.update();
        }
    }

    /*boolean dLeft = false;
    boolean dRight = false;
    boolean dDown = false;

    public void driveChassisWithController() {
        xPower = gamepad1.left_stick_x * Constants.DRIVE_SPEED_MULTIPLIER * (1 - gamepad1.left_trigger * 0.5);
        yPower = gamepad1.left_stick_y * Constants.DRIVE_SPEED_MULTIPLIER * (1 - gamepad1.left_trigger * 0.5);
        tPower = gamepad1.right_stick_x * Constants.DRIVE_SPEED_MULTIPLIER * (1 - gamepad1.left_trigger * 0.5);

        // case for driving the robot left and right
        if (Math.abs(Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x))) < Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(xPower, 0.0, tPower);

            // case for driving the robot forwards and backwards
        } else if (Math.abs(Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x))) > Constants.DRIVE_DEADZONE_DEGREES) {
            driveWithIMU(0.0, yPower, tPower);

            // case for if the deadzone limits are passed, the robot drives normally
        } else {
            driveWithIMU(xPower, yPower, tPower);
        }

        if (gamepad1.dpad_left && !dLeft) {
            IMUOriginalAngles.firstAngle = addAngle(*//* ඞ *//*IMUOriginalAngles.firstAngle, 90);
        } else if (gamepad1.dpad_right && !dRight) {
            IMUOriginalAngles.firstAngle = addAngle(*//* ඞ *//*IMUOriginalAngles.firstAngle, -90);
        } else if (gamepad1.dpad_down && !dDown) {
            IMUOriginalAngles.firstAngle = addAngle(*//* ඞ *//*IMUOriginalAngles.firstAngle, 180);
        }

        dLeft = gamepad1.dpad_left;
        dRight = gamepad1.dpad_right;
        dDown = gamepad1.dpad_down;
    }

    private float addAngle(float start, float add) {
        float newAngle = start + add;
        if (newAngle > 180.0) {
            newAngle -= 360.0;
        } else if (newAngle < -180.0) {
            newAngle += 360.0;
        }
        return newAngle;
    }*/
}
