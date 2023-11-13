package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad")
public class GamepadOpmode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        int currentTarget = 0;
        boolean holdPosition = false;
        int highPolePos = 1950;

        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.armMotor.getCurrentPosition();

            if (gamepad1.left_trigger > 0d) { // raise lift
                robot.armMotor.setTargetPosition(highPolePos);
                robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(gamepad1.left_trigger);
                currentTarget = currentArmPosition;
            } else if (gamepad1.right_trigger > 0d && // lower lift
                    robot.limitSwitch.getState() == true) { // when limit sensor not pressed
                double powerScale = currentArmPosition > 800 ? 1d : 0.4d;
                robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-gamepad1.right_trigger * powerScale);
                currentTarget = currentArmPosition;
                holdPosition = false;
            } else { // stop lift
                if (holdPosition == false && robot.limitSwitch.getState() == false) { // limit sensor pressed
                    robot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentTarget = 0;
                } else {
                    robot.armMotor.setTargetPosition(currentTarget);
                    robot.setArmMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(currentArmPosition > 100 ? 0.8d : 0.3d);
                }
            }
            //endregion

            //region grabber servo control
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    robot.toggleGrabber();
                }
            } else {
                a_pressed = false;
            }
            //endregion

            //region drivetrain control
            double leftStickY = -gamepad1.left_stick_y;  // Invert if necessary
            double rightStickY = -gamepad1.right_stick_y;  // Invert if necessary
            double drive;

            if (leftStickY >= 0 && rightStickY >= 0) {
                // Both inputs are positive, so choose the maximum positive value.
                drive = Math.max(leftStickY, rightStickY);
            } else if (leftStickY <= 0 && rightStickY <= 0) {
                // Both inputs are negative, so choose the minimum negative value.
                drive = Math.min(leftStickY, rightStickY);
            } else {
                // The inputs have different signs, so set drive to the sum of both.
                drive = leftStickY + rightStickY;
            }
            double turn = gamepad1.left_stick_x;
            double side = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                drive = 0.2d;
            } else if (gamepad1.dpad_down) {
                drive = -0.2d;
            } else if (gamepad1.dpad_left) {
                turn = -0.2d;
            } else if (gamepad1.dpad_right) {
                turn = 0.2d;
            }

            double leftFrontPower = drive + turn + side;
            double leftBackPower = drive + turn - side;
            double rightFrontPower = drive - turn - side;
            double rightBackPower = drive - turn + side;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            robot.setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            //endregion

            telemetry.update();
        }
    }
}
