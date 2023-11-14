package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize robot class
        Robot robot = new Robot(hardwareMap, this, telemetry, true);

        //getting motors and servos from config

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        //TODO: refactor to left/right
        DcMotor lsBack = hardwareMap.dcMotor.get("lsBack");
        DcMotor lsFront = hardwareMap.dcMotor.get("lsFront");

        DcMotor lFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rFront = hardwareMap.dcMotor.get("fRight");
        DcMotor lBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rBack = hardwareMap.dcMotor.get("bRight");

        Servo clamp = hardwareMap.servo.get("holderClamp");
        Servo tray = hardwareMap.servo.get("arm");
        Servo launcher = hardwareMap.servo.get("planeLauncher");
        Servo lock = hardwareMap.servo.get("linearLocker");
        Servo spikeServo = hardwareMap.servo.get("spikeServo");

        //setting motor direction
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //set motor zero power behavior
        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set run mode
        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double clampPos;
        double trayPos;
        boolean hangingMode;
        int polarity;

        waitForStart();

        //default positions
        clampPos = 0.5;
        trayPos = 0.594;

        hangingMode = false;
        polarity = 1;

        //set launcher and lock positions at start
        launcher.setPosition(0.25);
        lock.setPosition(0.5);

        while (opModeIsActive()) {

            //GAME PAD 2 CONTROLS NOT DRIVER

            double remainingDistanceHigh = 300 - lsFront.getCurrentPosition();
            double remainingDistanceMid = 200 - lsFront.getCurrentPosition();
            double remainingDistanceLow = 100 - lsFront.getCurrentPosition();
            double remainingDistanceZero = -1 * lsFront.getCurrentPosition();

            //gamepad 1 dpad moves linear slide to a set position
            if (gamepad1.dpad_up && remainingDistanceHigh > 10) {

                lsFront.setPower(remainingDistanceHigh * 0.002);
                lsBack.setPower(remainingDistanceHigh * 0.002);

            } else if (gamepad1.dpad_right && remainingDistanceMid > 10) {

                lsFront.setPower(remainingDistanceMid * 0.002);
                lsBack.setPower(remainingDistanceMid * 0.002);

            } else if (gamepad1.dpad_left && remainingDistanceLow > 10) {

                lsFront.setPower(remainingDistanceLow * 0.002);
                lsBack.setPower(remainingDistanceLow * 0.002);

            } else if (gamepad1.dpad_down && remainingDistanceZero > 10) {

                lsFront.setPower(remainingDistanceZero * 0.002);
                lsBack.setPower(remainingDistanceZero * 0.002);

            }

            //gamepad 2 dpad locks and unlocks the lock
            if (gamepad2.dpad_up) {
                lock.setPosition(0.3);
            } else if (gamepad2.dpad_down) {
                lock.setPosition(0.5);
            }

            //gamepad 2 B changes amount of linear slide motor power (for endgame hanging)
            if (gamepad2.b) {
                hangingMode = true;
                //turn on hanging mode to increase power for linear slides
            }

            if (!hangingMode) {
                //if not hanging, power less
                if (-gamepad2.left_stick_y > 0) {
                    lsBack.setPower(0.5);
                    lsFront.setPower(0.5);
                } else if (-gamepad2.left_stick_y < 0) {
                    lsBack.setPower(-0.5);
                    lsFront.setPower(-0.5);
                } else {
                    lsBack.setPower(0);
                    lsFront.setPower(0);
                }
            } else if (hangingMode) {
                //if hanging, power more
                if (-gamepad2.left_stick_y > 0) {
                    lsBack.setPower(1);
                    lsFront.setPower(1);
                } else if (-gamepad2.left_stick_y < 0) {
                    lsBack.setPower(-1);
                    lsFront.setPower(-1);
                } else {
                    lsBack.setPower(0);
                    lsFront.setPower(0);
                }
            }

            //if linear slide motor's # of ticks is under 4, set it to down
            if (lsFront.getCurrentPosition() < 4) {
                trayPos = 0.594;
            }

            //intake
            if (gamepad2.left_trigger > 0) {
                intake.setPower(0.4);
                clampPos = 0.4;
                //if intake button held, keep holder open
            } else if (gamepad2.left_bumper) {
                //reversed intake
                intake.setPower(-0.6);
                clampPos = 0.4;
            } else {
                if ((Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) && !gamepad2.left_bumper) {
                    //if robot moving, keep holder closed
                    clampPos = 0.5;
                }
                intake.setPower(0);
            }

            //right bumper and trigger of gamepad 2 open and close clamp
            if (gamepad2.right_bumper) {
                clampPos -= 0.025;
                //open
            } else if (gamepad2.right_trigger > 0) {
                clampPos += 0.05;
                //close
            }

            //gamepad 2 right stick manually pivots tray
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                trayPos -= -(gamepad2.right_stick_y / (1 / 0.004));
            }

            //set limits on the holder clamp
            if (clampPos > 1) {
                clampPos = 1;
            } else if (clampPos < 0.43) {
                clampPos = 0.43;
            }

            //set limits on how much the tray can pivot
            if (trayPos > 1) {
                trayPos = 1;
            } else if (trayPos < 0) {
                trayPos = 0;
            }

            //set intake/outtake positions for tray
            if (gamepad2.a) {
                trayPos = 0.594; //down (intake) position
            }
            if (gamepad2.y) {
                trayPos = 0.855; //up (outtake) position
            }

            //the earlier conditionals set variables based on what was pressed
            //here the servos are actually set to those variables
            clamp.setPosition(clampPos);
            tray.setPosition(trayPos);

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
                launcher.setPosition(0.7);
            }

            if (gamepad1.x) {
                if (robot.isRedAlliance) {
                    robot.setHeading(-90, 0.75);
                } else {
                    robot.setHeading(90, 0.75);
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
}
