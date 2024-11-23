package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleop_live_2controllers", group="Exercises")
//@Disabled
public class Teleop_live_2controllers extends LinearOpMode {
    BHI260IMU imu;
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;
    DcMotor m0 = null;
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    Servo s1 = null;
    Servo s2 = null;
    Servo s3 = null;
    CRServo s4 = null;
    Servo s5 = null;
    Servo s6 = null;
    Servo s12 = null;
    double left1Y, right1Y, left1X, right1X;
    double left2Y, right2Y, left2X, right2X;
    boolean flag_correction = true;
    boolean intake_constant = false;
    boolean gamepad1x_previous = false;
    boolean gamepad1RB_previous = false;
    boolean gamepad1LB_previous = false;
    boolean gamepad2RB_previous = false;
    boolean gamepad2LB_previous = false;
    boolean gamepad1dpadDown_previous = false;
    boolean gamepad1dpadRight_previous = false;
    boolean gamepad1dpadLeft_previous = false;

    double m2Power, blPower, flPower, brPower, frPower;
    static final double DEADZONE = 0.1;

    private double clampDeadzone(double val) {
        if (Math.abs(val) < DEADZONE) {
            return 0;
        } else {
            return val;
        }
    }


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        // Variable Initialization
        BHI260IMU.Parameters myIMUParameters;
        Orientation myRobotOrientation;

        // IMU in the control hub
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        // Start imu initialization
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        telemetry.addData("Gyro Status", "Initialized");
        telemetry.update();

        // wheel motors
        bl = hardwareMap.get(DcMotor.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m0 = hardwareMap.get(DcMotor.class, "M0");
        m1 = hardwareMap.get(DcMotor.class, "M1");
        m2 = hardwareMap.get(DcMotor.class, "M2");
        m3 = hardwareMap.get(DcMotor.class, "M3");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(CRServo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");
        s6 = hardwareMap.get(Servo.class, "s6");
        s12 = hardwareMap.get(Servo.class, "s12");
        s1.setDirection(Servo.Direction.FORWARD);
//        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.REVERSE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initial settings
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
        s4.setPower(0);
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setPosition(0); // open

        // Start opMode
        telemetry.addData("Mode", "waiting");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {

            // initiate the left and right variables
            left1Y = this.clampDeadzone(gamepad1.left_stick_y * -1);
            left1X = this.clampDeadzone(gamepad1.left_stick_x);
            right1Y = this.clampDeadzone(gamepad1.right_stick_y * -1);
            right1X = this.clampDeadzone(gamepad1.right_stick_x);

            left2Y = this.clampDeadzone(gamepad2.left_stick_y * -1);
            left2X = this.clampDeadzone(gamepad2.left_stick_x);
            right2Y = this.clampDeadzone(gamepad2.right_stick_y * -1);
            right2X = this.clampDeadzone(gamepad2.right_stick_x);


            //Linear Slide movement
            if (gamepad1.a && gamepad1.b) {
                m2Power = 0;
            } else if (gamepad1.b) {
                m2Power = 1;
            } else if (gamepad1.a) {
                m2Power = -1;
            } else {
                m2Power = 0.05;
            }

//
////            // Intake
            if (gamepad1.x != gamepad1x_previous && gamepad1.x ) {
                intake_constant = !intake_constant;
                telemetry.addData("intake",intake_constant);
                telemetry.update();
//                s1.setDirection(Servo.Direction.FORWARD);


                if (intake_constant) {
                    s1.setPosition(0);
                    //Extend
                } else {
                    s1.setPosition(1);
                    //Retract
                }
            }
            gamepad1x_previous = gamepad1.x;


            //Intake arm motor
            if (gamepad1.x) {
                s12.setPosition(0.5);
                s6.setPosition(0.5);
            }


            if (gamepad1.y) {

            }





                //specimen claw servo
                if (gamepad1.right_bumper != gamepad1RB_previous && gamepad1.right_bumper) {
                    s5.setDirection(Servo.Direction.FORWARD);
                    s5.setPosition(1); // open
                }
                gamepad1RB_previous = gamepad1.right_bumper;

                if (gamepad1.left_bumper != gamepad1LB_previous && gamepad1.left_bumper) {
                    s5.setDirection(Servo.Direction.REVERSE);
                    s5.setPosition(1); //close
                }
                gamepad1LB_previous = gamepad1.left_bumper;


                if (gamepad1.dpad_down != gamepad1dpadDown_previous && gamepad1.dpad_down) {
                    s12.setPosition(1);
                    sleep(100);
                    s12.setPosition(0.5);
                    s6.setPosition(0.75);
                    s12.setPosition(1);
                    sleep(100);
                    s12.setPosition(0.5);
                    s6.setPosition(0.75);
                }
                gamepad1dpadDown_previous = gamepad1.dpad_down;


                if (gamepad1.dpad_right != gamepad1dpadRight_previous && gamepad1.dpad_right) {
                    s12.setPosition(0.5);
                    s6.setPosition(1);
                }
                gamepad1dpadRight_previous = gamepad1.dpad_right;

                if (gamepad1.dpad_left != gamepad1dpadLeft_previous && gamepad1.dpad_left) {
                    s12.setPosition(1);
                    sleep(100);
                    s12.setPosition(0.5);
                    s6.setPosition(0.5);
                    s12.setPosition(1);
                    sleep(100);
                    s12.setPosition(0.5);
                    s6.setPosition(0.5);
                }
                gamepad1dpadLeft_previous = gamepad1.dpad_left;


                //intake wheel
                if (gamepad2.right_trigger > 0.2) {
                    s4.setPower(1);
                }

                if (gamepad2.left_trigger > 0.2) {
                    s4.setPower(-1);
                }

                if (gamepad2.dpad_up) {
                    s4.setPower(0);
                }


                //Basket
                if (gamepad1.right_trigger > 0.2) {
                    s3.setDirection(Servo.Direction.FORWARD);
                    s3.setPosition(0.5); //up
                }


                if (gamepad1.left_trigger > 0.2) {
                    s3.setDirection(Servo.Direction.FORWARD);
                    s3.setPosition(1); //down
                }


                //movement
                boolean leftStickActive = (left2X != 0) || (left2Y != 0);
                boolean rightStickActive = right2X != 0;
                //Forwards/Backward for gamepad 1
                if (leftStickActive == rightStickActive) {
                    flPower = 0;
                    frPower = 0;
                    brPower = 0;
                    blPower = 0;


                } else if (leftStickActive) {

                    double THRESH_WM_POWER = 1.0; // max abs wheel power
                    myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double correction = myRobotOrientation.thirdAngle / 180.0;
                    correction = (10.0 * correction * Math.abs(left2Y) / THRESH_WM_POWER);
                    if (flag_correction == false) {
                        correction = 0;
                    }
                    correction = 0;
                    double maxPow = THRESH_WM_POWER;
                    double flPow = left2Y + left2X + correction;
                    maxPow = Math.max(maxPow, Math.abs(flPow));
                    double blPow = left2Y - left2X + correction;
                    maxPow = Math.max(maxPow, Math.abs(blPow));
                    double frPow = left2Y - left2X - correction;
                    maxPow = Math.max(maxPow, Math.abs(frPow));
                    double brPow = left2Y + left2X - correction;
                    maxPow = Math.max(maxPow, Math.abs(brPow));
                    flPow = (flPow / maxPow) * THRESH_WM_POWER;
                    blPow = (blPow / maxPow) * THRESH_WM_POWER;
                    frPow = (frPow / maxPow) * THRESH_WM_POWER;
                    brPow = (brPow / maxPow) * THRESH_WM_POWER;

                    flPower = (Range.clip(flPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                    blPower = (Range.clip(blPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                    frPower = (Range.clip(frPow, -THRESH_WM_POWER, THRESH_WM_POWER));
                    brPower = (Range.clip(brPow, -THRESH_WM_POWER, THRESH_WM_POWER));

                    telemetry.addData("first Angle", myRobotOrientation.firstAngle);
                    telemetry.addData("second Angle", myRobotOrientation.secondAngle);
                    telemetry.addData("third Angle", myRobotOrientation.thirdAngle);
                    telemetry.addData("correction", correction);
                    telemetry.addData("leftY", left1Y);
                    telemetry.addData("leftX", left1X);
                    telemetry.addData("flPow", flPow);
                    telemetry.addData("blPow", blPow);
                    telemetry.addData("frPow", frPow);
                    telemetry.addData("brPow", brPow);
                    telemetry.addData("fl Enc Count", fl.getCurrentPosition());
                    telemetry.addData("bl Enc Count", bl.getCurrentPosition());
                    telemetry.addData("fr Enc Count", fr.getCurrentPosition());
                    telemetry.addData("br Enc Count", br.getCurrentPosition());
                    telemetry.update();
                    telemetry.update();
                } else {
                    //rightstick active
                    double THRESH_WM_POWER_FORTURN = 0.8;
                    flPower = (Range.clip(right2X * 0.6, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                    blPower = (Range.clip(right2X, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                    frPower = (Range.clip(right2X * 0.6, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                    brPower = (Range.clip(-right2X, -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN));
                    myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    telemetry.addData("THIS IS THE ANGLE", myRobotOrientation.thirdAngle);
                    telemetry.update();
                    //imu.resetYaw();
                    idle();
                }


                // DPad - down gamepad 1 and 2
//            if (gamepad1.dpad_down) {
//                imu.resetYaw();
//                bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
//                fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
//
//                bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            if (gamepad2.dpad_down) {
//                imu.resetYaw();
//                bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
//                fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
//
//                bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }

                //DPad - up for gamepad 1 and 2
//            if (gamepad1.dpad_up) {
//                flag_correction = !flag_correction;
//                boolean flag_2 = flag_correction;
//                flag_correction = flag_2;
//                sleep(1000);
//                telemetry.addData("Correction status", flag_correction);
//                telemetry.update();
//            }
//            if (gamepad2.dpad_up) {
//                flag_correction = !flag_correction;
//                boolean flag_2 = flag_correction;
//                flag_correction = flag_2;
//                sleep(1000);
//                telemetry.addData("Correction status", flag_correction);
//                telemetry.update();
//            }

                //sideways: gamepad 1 and 2
//            if (gamepad1.dpad_left || gamepad1.dpad_right) {
//                double directionFactor = 1;
//                if (gamepad1.dpad_right) {
//                    directionFactor = -1;
//                }
//                double THRESH_WM_POWER_SIDEWAYS = 0.8;
//                double frEC = fr.getCurrentPosition();
//                double blEC = bl.getCurrentPosition();
//                double flEC = fl.getCurrentPosition();
//                double brEC = br.getCurrentPosition();
//                double frCorr = 1;
//                double blCorr = 1;
//                double flCorr = 1;
//                double brCorr = 1;
//                if (frEC != 0 && (flag_correction == true)) {
//                    frEC = Math.abs(frEC);
//                    double refEC = frEC;
//                    blEC = Math.abs(blEC);
//                    refEC = Math.min(refEC, blEC);
//                    flEC = Math.abs(flEC);
//                    refEC = Math.min(refEC, flEC);
//                    brEC = Math.abs(brEC);
//                    refEC = Math.min(refEC, brEC);
//                    if (refEC == 0) {
//                        refEC = 1;
//                    }
//                    frCorr = refEC / frEC;
//                    blCorr = refEC / blEC;
//                    flCorr = refEC / flEC;
//                    brCorr = refEC / brEC;
//                }
//                double flPow = directionFactor * -THRESH_WM_POWER_SIDEWAYS * flCorr;
//                double blPow = directionFactor * THRESH_WM_POWER_SIDEWAYS * blCorr;
//                double frPow = directionFactor * THRESH_WM_POWER_SIDEWAYS * frCorr;
//                double brPow = directionFactor * -THRESH_WM_POWER_SIDEWAYS * brCorr;
//                fl.setPower(flPow);
//                bl.setPower(blPow);
//                fr.setPower(frPow);
//                br.setPower(brPow);
//
//                telemetry.addData("fl Pow", flPow);
//                telemetry.addData("bl Pow", blPow);
//                telemetry.addData("fr Pow", frPow);
//                telemetry.addData("br Pow", brPow);
//                telemetry.addData("fl Enc Count", fl.getCurrentPosition());
//                telemetry.addData("bl Enc Count", bl.getCurrentPosition());
//                telemetry.addData("fr Enc Count", fr.getCurrentPosition());
//                telemetry.addData("br Enc Count", br.getCurrentPosition());
//                telemetry.update();
//            }

                // turning for gamepad 1 and 2


                m2.setPower(m2Power);
                bl.setPower(blPower);
                br.setPower(brPower);
                fl.setPower(flPower);
                fr.setPower(frPower);

            }

        }

    public void slideUp(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() > -encoderAbsCounts){
            m2.setPower(power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0.05); // set power to 0 so the motor stops running

    }

    public void slideDown(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() < encoderAbsCounts){
            m2.setPower(-power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0); // set power to 0 so the motor stops running

    }


    }
