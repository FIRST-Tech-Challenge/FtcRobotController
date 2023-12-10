package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Full", group = "CenterStage")
public class CSFullTeleOp extends CSMethods {

    // Declare OpMode members for each of the 4 motors.
    boolean lBack = false;
    boolean rBack = false;
    boolean a = false;

    @Override
    public void runOpMode() {
        setup(true);

        double slow = 1.0;
        double turnSpeed = 2.0;
        int targetEncoderValue = 0;
        double carWashPower = 1.0;

        while (opModeIsActive()) {
            double max;
            if (false) {
                slow = 0.35;
                turnSpeed = 1.25;
            } else {
                slow = 0.75;
                turnSpeed = 2.5;
            }

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            axial = -gamepad1.left_stick_y * slow;
            lateral = gamepad1.left_stick_x * 1.0 * slow;
            yaw = gamepad1.right_stick_x * turnSpeed;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(leftBackPower);
            rb.setPower(rightBackPower);

            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                pixelLiftingMotor.setTargetPosition(1000);
                pixelLiftingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_down && !gamepad2.dpad_up) {
                pixelLiftingMotor.setTargetPosition(0);
                pixelLiftingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                pixelLiftingMotor.setTargetPosition(pixelLiftingMotor.getCurrentPosition());
                pixelLiftingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            if (gamepad2.a || gamepad2.x) {
                carWashMotor.setPower(carWashPower);
            }

            if (gamepad1.left_bumper && !lBack) {
                lBack = true;
                if (pixelBackServo.getPosition() == 1) {
                    pixelBackServo.setPosition(0);
                } else {
                    pixelBackServo.setPosition(1);
                }
            } else if (!gamepad1.left_bumper) {
                lBack = false;
            }

            if (gamepad1.a && !a) {
                a = true;
                if (trayTiltingServo.getPosition() == 1) {
                    trayTiltingServo.setPosition(0);
                } else {
                    trayTiltingServo.setPosition(1);
                }
            } else if (!gamepad1.a) {
                a = false;
            }

            if (gamepad1.right_bumper && !rBack) {
                rBack = true;
                if (pixelFrontServo.getPosition() == 1) {
                    pixelFrontServo.setPosition(0);
                } else {
                    pixelFrontServo.setPosition(1);
                }
            } else if (!gamepad1.right_bumper) {
                rBack = false;
            }

            if (gamepad2.b || gamepad2.y) {
                carWashMotor.setPower(-carWashPower);
            }

            if ((gamepad1.left_trigger > 0.85) && (gamepad1.right_trigger > 0.85) && runtime.seconds() > 90) {
                droneServo.setPosition(1);
            } else if ((gamepad1.left_trigger > 0.85) && (gamepad1.right_trigger > 0.85) && gamepad1.left_bumper && gamepad1.right_bumper) {
                droneServo.setPosition(1);
            }

            if (!gamepad2.a && !gamepad2.b && !gamepad2.y && !gamepad2.x) {
                carWashMotor.setPower(0);
            }
            telemetry.addData("Run Time", runtime.toString());
            if (runtime.seconds() > 90) {
                telemetry.addData("Game Phase", "End Game");
            } else {
                telemetry.addData("Game Phase", "Driver Controlled");
            }
            telemetry.update();

            sleep(25);
        }
    }
}
