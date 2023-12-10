package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Full", group = "CenterStage")
public class CSFullTeleOp extends CSMethods {

    // Declare OpMode members for each of the 4 motors.
    boolean lBack = false;
    boolean rBack = false;
    boolean a = false;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    final double speedMultiplier = 0.75;
    final double baseTurnSpeed = 2.5;
    double slowdownMultiplier = 0.0;


    @Override
    public void runOpMode() {
        setup(true);

        double carWashPower = 1.0;

        while (opModeIsActive()) {
            slowdownMultiplier = (1.0 - gamepad1.right_trigger);

            axial = ((-gamepad1.left_stick_y * speedMultiplier) * slowdownMultiplier);
            lateral = ((gamepad1.left_stick_x * speedMultiplier) * slowdownMultiplier);
            yaw = ((gamepad1.right_stick_x * baseTurnSpeed) * slowdownMultiplier);

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

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
            lf.setPower(leftFrontPower);
            rf.setPower(rightFrontPower);
            lb.setPower(leftBackPower);
            rb.setPower(rightBackPower);

            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                if (pixelLiftingMotor.getCurrentPosition() < 2000){
                    pixelLiftingMotor.setPower(1);
                }
                else {
                    pixelLiftingMotor.setPower(0);
                }
            }

            if (gamepad2.dpad_down && !gamepad2.dpad_up) {
                if (pixelLiftingMotor.getCurrentPosition() > 0){
                    pixelLiftingMotor.setPower(-1);
                }
                else {
                    pixelLiftingMotor.setPower(0);
                }
            }

            if (!gamepad2.dpad_up && !gamepad2.dpad_down || gamepad2.dpad_down && gamepad2.dpad_up) {
                pixelLiftingMotor.setPower(0);
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
                if (trayTiltingServo.getPosition() == 0.5) {
                    trayTiltingServo.setPosition(0);
                } else {
                    trayTiltingServo.setPosition(0.5);
                }
            } else if (!gamepad1.a) {
                a = false;
            }

            /*if (gamepad1.right_bumper && !rBack) {
                rBack = true;
                if (pixelFrontServo.getPosition() == 1) {
                    pixelFrontServo.setPosition(0);
                } else {
                    pixelFrontServo.setPosition(1);
                }
            } else if (!gamepad1.right_bumper) {
                rBack = false;
            }*/

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
