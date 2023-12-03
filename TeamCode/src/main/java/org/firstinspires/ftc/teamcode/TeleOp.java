package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    Robot robot;
    double trayPos;
    private static double TRAY_DOWN_POS = 0.424677;
    private static double TRAY_UP_POS = 0.1;
    double clampPos;
    double hookPos = 0.49;
    private static double CLAMP_CLOSE_POS = 0.522;
    private static double CLAMP_OPEN_POS = 0.4724;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize robot class
        robot = new Robot(hardwareMap, this, telemetry, true, false);
        robot.setUpDrivetrainMotors();
        robot.setUpIntakeOuttake();

        boolean hangingMode;
        int polarity;

        Log.d("vision", "runOpMode: heading " + robot.getCurrentHeading());

        waitForStart();

        //default positions
        clampPos = CLAMP_CLOSE_POS;
        trayPos = TRAY_DOWN_POS;

        hangingMode = false;
        polarity = 1;

        //set launcher and lock positions at start
        //robot.planeLauncher.setPosition(0.25);
        robot.hook.setPosition(hookPos);

        while (opModeIsActive()) {

            //GAME PAD 2 CONTROLS NOT DRIVER

            double remainingDistanceHigh = 300 - robot.lsFront.getCurrentPosition();
            double remainingDistanceMid = 200 - robot.lsFront.getCurrentPosition();
            double remainingDistanceLow = 100 - robot.lsFront.getCurrentPosition();
            double remainingDistanceZero = -1 * robot.lsFront.getCurrentPosition();

            //gamepad 1 dpad moves linear slide to a set position
            if (gamepad1.dpad_up && remainingDistanceHigh > 10) {
                robot.lsFront.setPower(remainingDistanceHigh * 0.002);
                robot.lsBack.setPower(remainingDistanceHigh * 0.002);
            } else if (gamepad1.dpad_right && remainingDistanceMid > 10) {
                robot.lsFront.setPower(remainingDistanceMid * 0.002);
                robot.lsBack.setPower(remainingDistanceMid * 0.002);
            } else if (gamepad1.dpad_left && remainingDistanceLow > 10) {
                robot.lsFront.setPower(remainingDistanceLow * 0.002);
                robot.lsBack.setPower(remainingDistanceLow * 0.002);
            } else if (gamepad1.dpad_down && remainingDistanceZero > 10) {
                robot.lsFront.setPower(remainingDistanceZero * 0.002);
                robot.lsBack.setPower(remainingDistanceZero * 0.002);
            }

            //gamepad 2 dpad locks and unlocks the lock
            if (gamepad2.dpad_up) {
                robot.hook.setPosition(0.27);
                telemetry.addData("hook pos", hookPos);
                //robot.hook.setPosition(0.3);//0.3
            } else if (gamepad2.dpad_down) {
                robot.hook.setPosition(0.49);
                telemetry.addData("hook pos", hookPos);
                //robot.hook.setPosition(0.5);//0.5
            }

            //gamepad 2 B changes amount of linear slide motor power (for endgame hanging)
            if (gamepad2.b) {
                hangingMode = true;
                //turn on hanging mode to increase power for linear slides
            }

            if (!hangingMode) {
                //if not hanging, power less
                if (-gamepad2.left_stick_y > 0) {
                    robot.lsBack.setPower(0.5);
                    robot.lsFront.setPower(0.5);
                } else if (-gamepad2.left_stick_y < 0) {
                    robot.lsBack.setPower(-0.5);
                    robot.lsFront.setPower(-0.5);
                } else {
                    robot.lsBack.setPower(0);
                    robot.lsFront.setPower(0);
                }
            } else if (hangingMode) {
                //if hanging, power more
                if (-gamepad2.left_stick_y > 0) {
                    robot.lsBack.setPower(1);
                    robot.lsFront.setPower(1);
                } else if (-gamepad2.left_stick_y < 0) {
                    robot.lsBack.setPower(-1);
                    robot.lsFront.setPower(-1);
                } else {
                    robot.lsBack.setPower(0);
                    robot.lsFront.setPower(0);
                }
            }

            //intake
            if (gamepad2.left_trigger > 0) {
                robot.intake.setPower(-1);
                clampPos = CLAMP_OPEN_POS;
                //if intake button held, keep holder open
            } else if (gamepad2.left_bumper) {
                //reversed intake
                robot.intake.setPower(0.6);
                clampPos = CLAMP_OPEN_POS;
            } else {
                if ((Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) && !gamepad2.left_bumper) {
                    //if robot moving, keep holder closed
                    clampPos = CLAMP_CLOSE_POS; //TODO UNCOMMENT
                }
                robot.intake.setPower(0);
            }

            //right bumper and trigger of gamepad 2 open and close clamp
            if (gamepad2.right_bumper) {
                clampPos = 0.4724;
                telemetry.addData("clamp pos", clampPos);
                //open
            } else if (gamepad2.right_trigger > 0) {
                clampPos = 0.522;
                telemetry.addData("clamp pos", clampPos);

                //close
            }


            //gamepad 2 right stick manually pivots tray
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                trayPos -= -(gamepad2.right_stick_y / (1 / 0.004));
                telemetry.addData("tray pos", trayPos);
                //Log.d("tray debug", "runOpMode: trayPos = " + trayPos);
            }


            //set limits on the holder clamp
            /* if (clampPos > 1) {
                clampPos = 1;
            } else if (clampPos < CLAMP_OPEN_POS) {
                clampPos = CLAMP_OPEN_POS;
            } */ //TODO UNCOMMENT

            //set intake/outtake positions for tray
            if (gamepad2.a) {
                robot.trayToIntakePos();
            } else if (gamepad2.y) {
                robot.trayToOuttakePos();
            }

            //the earlier conditionals set variables based on what was pressed
            //here the servos are actually set to those variables
            robot.clamp.setPosition(clampPos);
            robot.tray.setPosition(trayPos);

            //GAMEPAD 1 CONTROLS DRIVER

            /*
            if (gamepad1.a) {
                spikeServo.setPosition(0.4);
            }
            if (gamepad1.b) {
                spikeServo.setPosition(0.2);
            }*/

            //gamepad 1 right bumper launches drone
            if (gamepad1.right_bumper) {
                robot.planeLauncher.setPosition(0.7);
            }

            if (gamepad1.x) {
                if (robot.isRedAlliance) {
                    robot.setHeading(90, 0.75);
                } else {
                    robot.setHeading(-90, 0.75);
                }
            }


            //y is intake at back, a is intake at front
            //TODO: test this and all polarity logic
            if (gamepad1.a) {
                polarity = 1;
            } else if (gamepad1.y) {
                polarity = -1;
            }

            //doubles for amount of input for straight, turning, and mecanuming variables
            double straight;
            double turning;
            double mecanuming;

            //setting forward and mecanum based on where the front is
            straight = gamepad1.left_stick_y * polarity * -0.75;
            mecanuming = gamepad1.left_stick_x * polarity * 0.75;

            //turning stays the same
            turning = gamepad1.right_stick_x * 0.75;

            //set powers using this input
            double fLeftPower = straight + turning + mecanuming;
            double fRightPower = straight - turning - mecanuming;
            double bLeftPower = straight + turning - mecanuming;
            double bRightPower = straight - turning + mecanuming;

            //scale powers
            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);
                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            robot.setMotorPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

            telemetry.addLine("position: " + trayPos);
            telemetry.addLine("slide position: " + robot.lsFront.getCurrentPosition());
            telemetry.update();
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }

    private void slidesToMid() {
        robot.moveLinearSlidesByTicksParallel(-2400);
    }

    private void oneButtonOuttake() {
        slidesToMid();
        trayPos = TRAY_UP_POS;
        robot.setServoPosBlocking(robot.tray, trayPos);
    }
}
