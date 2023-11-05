package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Gamepad")
public class GamepadOpMode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        int armLimit = 2000;

        double rollerVelocity = 0.0; // Initial velocity
        boolean isRollerRunning = false; // Track whether the roller is running
        boolean loweringInProgress = false; // Initialize a flag to track lowering state
        boolean isTimerRunning = false;
        long startTime = System.currentTimeMillis();
        double timeToFullPower = 2.0; // Time (in seconds) to reach full power
        double maxPower = 0.6d;

        robot.init(hardwareMap);

        double bowlPosition = robot.bowlServo.getPosition();

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("roller", () -> String.format("%.1f", robot.rollerMotor.getVelocity()));
        telemetry.addLine("arm | ")
                .addData("pos", () -> String.format("%d", robot.liftMotor.getCurrentPosition()))
                .addData("amp", () -> String.format("%.1f", robot.liftMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.addLine("sensor | ")
                .addData("limit", () -> String.format("%s", robot.limitSwitch.getState()))
                .addData("front", () -> String.format("%.1f cm", robot.frontSensor.getDistance(DistanceUnit.CM)));
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.liftMotor.getCurrentPosition();
            boolean limitSwitch = robot.limitSwitch.getState();

            if (gamepad1.left_trigger > 0d) { // raise lift
                if (limitSwitch) { // not touch

                    robot.setArmPosition(0.8d);
                    rollerVelocity = 0;
                    isRollerRunning = false;
                }
                robot.liftMotor.setTargetPosition(armLimit);
                robot.setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(gamepad1.left_trigger);
                loweringInProgress = false; // Reset the lowering state
            } else if (gamepad1.right_trigger > 0d &&  // lower lift
                    limitSwitch) { // Check if the limit switch is not pressed
                double powerScale = currentArmPosition > 800 ? 1d : 0.3d;
                robot.setLiftMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-gamepad1.right_trigger * powerScale);
                loweringInProgress = false; // Reset the lowering state
            } else {
                if (!loweringInProgress) { // stop lift
                    robot.liftMotor.setPower(0d);
                } else {
                    robot.setLiftMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    robot.liftMotor.setPower(-0.4d);
                }

                if (!limitSwitch) {
                    if (loweringInProgress) {
                        loweringInProgress = false;
                        robot.setArmPosition(1d);
                    }
                    robot.setLiftMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            if (gamepad1.x) {
                if (!x_pressed) {
                    x_pressed = true;
                    robot.beep();
                    robot.setArmPosition(0.8d);
                    robot.setBowlPosition(0d);
                    loweringInProgress = !loweringInProgress;
                }
            } else { // Reset the 'X' button press flag
                x_pressed = false;
            }
            //endregion

            //region a button
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    if (limitSwitch) {
                        robot.toggleArm();
                    }
                }
            } else {
                a_pressed = false;
            }
            //endregion

            //region b button
            if (gamepad1.b) {
                if (!b_pressed) {
                    b_pressed = true;
                    if (limitSwitch) {
                        robot.toggleBowl();
                    }
                }
            } else {
                b_pressed = false;
            }
            //endregion

            if (gamepad1.y) {
                if (!y_pressed) {
                    y_pressed = true;
                }
            }

            //region roller
            if (gamepad1.left_bumper) {
                if (!lb_pressed) {
                    if (isRollerRunning) {
                        // If the roller is running, stop it
                        rollerVelocity = 0.0;
                        isRollerRunning = false;
                    } else if (!limitSwitch) {
                        // If the roller is stopped, start it with positive velocity
                        rollerVelocity = 2500.0;
                        isRollerRunning = true;
                    }
                    lb_pressed = true;
                }
            } else {
                lb_pressed = false;
            }

            if (gamepad1.right_bumper) {
                if (!rb_pressed) {
                    if (isRollerRunning) {
                        // If the roller is running, stop it
                        rollerVelocity = 0.0;
                        isRollerRunning = false;
                    } else {
                        // If the roller is stopped, start it with negative velocity
                        rollerVelocity = -1000.0;
                        isRollerRunning = true;
                    }
                    rb_pressed = true;
                }
            } else {
                rb_pressed = false;
            }

            robot.rollerMotor.setVelocity(rollerVelocity);
            //endregion

            //region drivetrain control
            double sensitivity = 1.4d; // Adjust the sensitivity value as needed
            double deadzone = 0.05d; // Adjust the deadzone value as needed
            double drive = (-gamepad1.left_stick_y - gamepad1.right_stick_y) * 0.6d;
            double turn = gamepad1.left_stick_x * 0.6d;
            double side = gamepad1.right_stick_x;
            double frontDistance = robot.frontSensor.getDistance(DistanceUnit.CM);

            if (gamepad1.dpad_up) {
                drive = 0.2d;
            } else if (gamepad1.dpad_down) {
                drive = -0.2d;
            } else if (gamepad1.dpad_left) {
                turn = -0.2d;
            } else if (gamepad1.dpad_right) {
                turn = 0.2d;
            }
            // Check if any stick's x or y value is greater than 0.6
//            boolean isStickGreaterThan0_6 = Math.abs(drive) > 0.6 || Math.abs(turn) > 0.6 || Math.abs(side) > 0.6;

            // Calculate the elapsed time (in seconds) based on the condition
//            double elapsedTime = 0.0;
//            if (isStickGreaterThan0_6) {
//                // Implement a mechanism to start or update a timer when a stick value exceeds 0.6
//                if (!isTimerRunning) {
//                    startTime = System.currentTimeMillis();
//                    isTimerRunning = true;
//                }
//                elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0; // Convert to seconds
//
//                // Gradually increase maxPower from 0.6 to 1
//                maxPower = 0.6 + elapsedTime * (1.0 - 0.6) / timeToFullPower;
//            } else {
//                // Reset the timer and maxPower when no stick value is greater than 0.6
//                isTimerRunning = false;
//                maxPower = 0.6d;
//            }

            //            if (drive > 0d && frontDistance < 10d) {
            //                maxPower = Range.scale(frontDistance, 2d, 15d, 0.05d, 0.2d);
            //            }

            // Apply deadzone to the input values
//            if (Math.abs(drive) < deadzone) {
//                drive = 0.0;
//            }
//
//            if (Math.abs(turn) < deadzone) {
//                turn = 0.0;
//            }
//
//            if (Math.abs(side) < deadzone) {
//                side = 0.0;
//            }

            // Apply the power function to the input values
//            drive = Math.signum(drive) * Math.pow(Math.abs(drive), sensitivity);
//            turn = Math.signum(turn) * Math.pow(Math.abs(turn), sensitivity);
//            side = Math.signum(side) * Math.pow(Math.abs(side), sensitivity);

            double pLeftFront = Range.clip(drive + turn + side, -1.0, 1.0);
            double pLeftRear = Range.clip(drive + turn - side, -1.0, 1.0);
            double pRightFront = Range.clip(drive - turn - side, -1.0, 1.0);
            double pRightRear = Range.clip(drive - turn + side, -1.0, 1.0);

            // Scale the power values based on maxPower
//            pLeftFront = Range.scale(pLeftFront, -1.0, 1.0, -maxPower, maxPower);
//            pLeftRear = Range.scale(pLeftRear, -1.0, 1.0, -maxPower, maxPower);
//            pRightFront = Range.scale(pRightFront, -1.0, 1.0, -maxPower, maxPower);
//            pRightRear = Range.scale(pRightRear, -1.0, 1.0, -maxPower, maxPower);

            // Send calculated power to wheels
            robot.setDrivePower(pLeftFront, pLeftRear, pRightFront, pRightRear);
            //endregion

            gamepadUpdate();
            telemetry.addLine("motor | ")
                    .addData("lf", String.format("%.1f", pLeftFront))
                    .addData("lr", String.format("%.1f", pLeftRear))
                    .addData("rf", String.format("%.1f", pRightFront))
                    .addData("rr", String.format("%.1f", pRightRear));
            telemetry.addLine("control | ")
                    .addData("drive", String.format("%.1f", drive))
                    .addData("turn", String.format("%.1f", turn))
                    .addData("side", String.format("%.1f", side));
            telemetry.addLine("servo | ")
                    .addData("bowl", String.format("%.1f", bowlPosition))
                    .addData("arm", String.format("%.1f", robot.armPosition));
            telemetry.addData("timer", isTimerRunning);
            telemetry.update();
        }
    }
}
