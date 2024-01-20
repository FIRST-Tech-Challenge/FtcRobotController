package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Tele", group = "Pushbot")
public class tele extends OpMode {
    Hardware robot = new Hardware();
    double speedLimit;
    double y;
    double x;
    double rx;

    double denominator;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    int deploymentState = 0;
    int escapmentFingerPosition = 0;
    int launcherAngle = 0;
    double oldTime = 0;
    double escapementTime = 0;
    double angleTime = 0;

    double transferRPM;
    double oldTransferPosition;
    double oldTransferTime;


    int pixlePickerColorTime = 5000;

    Gamepad.LedEffect gamepadLaunchSequenceLed = new Gamepad.LedEffect.Builder()
            .addStep(100, 0, 0, 2000)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(0, 100, 0, 200)
            .addStep(100, 0, 0, 200)
            .addStep(100, 0, 100, 5000)
            .build();
    Gamepad.RumbleEffect gamepadLaunchSequenceRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(.3, .3, 2000)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .addStep(.5, 0, 200)
            .addStep(0, .5, 200)
            .build();


    public void init() {

        telemetry.addData("Robot:", "Initializing");
        telemetry.update();

        robot.init(hardwareMap);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.hook.setPosition(robot.hookDown);
        robot.droneAngle.setPosition(robot.droneAngleDown);
        robot.launcherRelease.setPosition(robot.launchClosed);

        telemetry.addData("Robot:", "Ready");
        telemetry.update();

    }

    public void start() {

    }

    public void loop() {

        if (gamepad1.right_trigger > .5) {
            speedLimit = 50;
        } else {
            speedLimit = 100;
        }

        double speedLimitValue = speedLimit / 100;

        if (gamepad1.left_trigger > .5) {
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
        } else {
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        }
        rx = gamepad1.right_stick_x;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = (y + x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        if ((Math.abs(gamepad1.right_stick_x) > 0.1) || (Math.abs(gamepad1.right_stick_y) > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1) || (Math.abs(gamepad1.left_stick_y) > 0.1)) {
            robot.leftDrive.setPower(frontLeftPower * speedLimitValue);
            robot.leftBackDrive.setPower(backLeftPower * speedLimitValue);
            robot.rightDrive.setPower(frontRightPower * speedLimitValue);
            robot.rightBackDrive.setPower(backRightPower * speedLimitValue);
        } else {
            robot.leftDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
        }

        /*
        if (robot.lift.getCurrentPosition() < 390 && !gamepad2.right_stick_button) {
            if (-gamepad2.right_stick_y <= 0) {
                if (robot.lift.getCurrentPosition() < 380) {
                    robot.lift.setPower(.7);
                } else {
                    robot.lift.setPower(0);
                }
            } else {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(-gamepad2.right_stick_y);
            }
        } else if (robot.liftDownSwitch.getVoltage() < .5) {
            if (-gamepad2.right_stick_y < 0) {
            robot.lift.setPower(0);
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(-gamepad2.right_stick_y);
            }
        } else {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(-gamepad2.right_stick_y);
        }
        */


        if (robot.liftDownSwitch.getVoltage() < .5) {
            if (-gamepad2.right_stick_y < 0) {
                robot.lift.setPower(0);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(-gamepad2.right_stick_y);
            }
        } else {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(-gamepad2.right_stick_y);
        }

        /** Stripper State **/


        switch (deploymentState) {
            case 0:
                deploymentState++;
                break;
            case 1:
                robot.stripper.setPosition(robot.stripperOpen);
                if (getRuntime() > oldTime + .2) {
                    if (gamepad2.left_trigger > .5) {
                        deploymentState++;
                        oldTime = getRuntime();
                    } else if (gamepad2.left_bumper) {
                        deploymentState = 1;
                        oldTime = getRuntime();
                    }
                }
                break;
            case 2:
                robot.stripper.setPosition(robot.stripperFirstRelease);
                if (getRuntime() > oldTime + .2) {
                    if (gamepad2.left_trigger > .5) {
                        deploymentState++;
                        oldTime = getRuntime();
                    } else if (gamepad2.left_bumper) {
                        deploymentState = 1;
                        oldTime = getRuntime();
                    }
                }
                break;
            case 3:
                robot.stripper.setPosition(robot.stripperSecondRelease);
                if (getRuntime() > oldTime + .75) {
                    deploymentState = 1;
                }
                break;
        }


        if (robot.lift.getCurrentPosition() < 650) {
            deploymentState = 1;
        }


        /** Winch **/

        if (gamepad2.dpad_down) {
            robot.hook.setPosition(robot.hookDown);
        }

        if (gamepad2.dpad_left) {
            robot.hook.setPosition(robot.winchAngleIntakeSide);
        }

        if (gamepad2.dpad_right) {
            robot.hook.setPosition(robot.winchAngleDeliverySide);
        }

        if (robot.winchDownSwitch.getVoltage() < .5) {
            if (-gamepad2.left_stick_y < 0) {
                robot.winch.setPower(0);
                robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.winch.setPower(-gamepad2.left_stick_y);
            }
        } else {
            robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.winch.setPower(-gamepad2.left_stick_y);
        }

        /** Drone Launcher **/
        if (gamepad2.y && (angleTime < (getRuntime() + 2))) {
            angleTime = getRuntime();
            launcherAngle++;
        }

        switch (launcherAngle) {
            case 0:
                launcherAngle++;
                break;
            case 1:
                robot.droneAngle.setPosition(robot.droneAngleDown);
                break;
            case 2:
                robot.droneAngle.setPosition(robot.droneAngleUp);
                break;
            case 3:
                launcherAngle = 1;
        }

        if (gamepad2.x && gamepad2.b) {
            robot.launcherRelease.setPosition(robot.launchOpen);
        } else {
            robot.launcherRelease.setPosition(robot.launchClosed);
        }

        /** Intake/Transfer **/

//        switch (escapmentFingerPosition) {
//            case 0:
//                escapementTime = getRuntime();
//                escapmentFingerPosition++;
//                break;
//            case 1:
//                robot.escapementFinger.setPosition(-.1);
//                if (getRuntime() > escapementTime + .5) {
//                    robot.escapementFinger.setPosition(0);
//                    escapmentFingerPosition++;
//                }
//                break;
//            case 2:
//                if (gamepad2.dpad_down) {
//                    escapementTime = getRuntime();
//                    escapmentFingerPosition++;
//                    break;
//                case 1:
//                    robot.escapementFinger.setPosition(-.1);
//                    if (getRuntime() > escapementTime + .5) {
//                        robot.escapementFinger.setPosition(0);
//                        escapmentFingerPosition++;
//                    }
//                    break;
//                case 2:
//                    if (gamepad2.dpad_down) {
//                        escapementTime = getRuntime();
//                        escapmentFingerPosition++;
//                    }
//                    break;
//                case 3:
//                    robot.escapementFinger.setPosition(.1);
//                    if (getRuntime() > escapementTime + .5) {
//                        robot.escapementFinger.setPosition(0);
//                        escapmentFingerPosition++;
//                    }
//                    break;
//                case 4:
//                    if (gamepad2.dpad_down) {
//                        escapmentFingerPosition++;
//                    }
//                    break;
//                case 5:
//                    escapmentFingerPosition = 0;
//                    break;
//            }


        if (gamepad2.right_trigger > .5) {
            robot.transfer.setPower(.7);
            robot.intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.right_bumper) {
            robot.transfer.setPower(-1);
            robot.intake.setPower(-1);
        } else {
            robot.transfer.setPower(0);
            robot.intake.setPower(0);
        }

        if (gamepad1.x) {
            gamepad1.setLedColor(255, 0, 255, pixlePickerColorTime);
            gamepad2.setLedColor(255, 0, 255, pixlePickerColorTime);
        }

        if (gamepad1.y) {
            gamepad1.setLedColor(0, 255, 0, pixlePickerColorTime);
            gamepad2.setLedColor(0, 255, 0, pixlePickerColorTime);
        }

        if (gamepad1.b) {
            gamepad1.setLedColor(249, 100, 0, pixlePickerColorTime);
            gamepad2.setLedColor(249, 100, 0, pixlePickerColorTime);
        }

        if (gamepad1.a) {
            gamepad1.setLedColor(255, 255, 255, pixlePickerColorTime);
            gamepad2.setLedColor(255, 255, 255, pixlePickerColorTime);
        }

        if (gamepad2.right_bumper && gamepad1.left_bumper) {
            gamepad1.runLedEffect(gamepadLaunchSequenceLed);
            gamepad2.runLedEffect(gamepadLaunchSequenceLed);
            gamepad1.runRumbleEffect(gamepadLaunchSequenceRumble);
            gamepad2.runRumbleEffect(gamepadLaunchSequenceRumble);
        }

        telemetry.addData("Right Distance", robot.rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", robot.leftDistance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

}
