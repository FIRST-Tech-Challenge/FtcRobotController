package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "REAL TELEOP", group = "Linear OpMode")
//@Disabled
public class Official extends LinearOpMode {

    /* Declare OpMode members. */
    public Servo intakeClaw = null;
    public Servo clawPivot = null;
    public Servo wrist = null;
    public Servo intakeElbow = null;

    public Servo outtakeClaw = null;
    public Servo outtakeElbow = null;

    public Servo intakeSlide1 = null;
    public Servo intakeSlide2 = null;
    private boolean elbowIsDown = false;
    private boolean outtakeIsDown = false;
    private boolean outtakeIsFlat = false;
    private boolean slidesIsUp = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor outtakeSlide1;
    private DcMotor outtakeSlide2;
    private final int[] slidePosition = {0};

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private IMU imu;

    private double sethControlThing;

    @Override
    public void runOpMode() {

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Motor0");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor2");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //slide motors
        outtakeSlide1 = hardwareMap.get(DcMotor.class, "Motor5");
        outtakeSlide2 = hardwareMap.get(DcMotor.class, "Motor6");
        outtakeSlide1.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define and initialize ALL installed servos.
        intakeClaw = hardwareMap.get(Servo.class, "0");
        clawPivot = hardwareMap.get(Servo.class, "1");
        wrist = hardwareMap.get(Servo.class, "2");
        intakeElbow = hardwareMap.get(Servo.class, "3");
        intakeSlide2 = hardwareMap.get(Servo.class, "4");
        intakeSlide1 = hardwareMap.get(Servo.class, "5");

        outtakeElbow = hardwareMap.get(Servo.class, "7");
        outtakeClaw = hardwareMap.get(Servo.class, "6");

        outtakeClaw.setPosition(Values.outakeclawOpen);
        outtakeElbow.setPosition(Values.outtakeElbowDown);

        clawPivot.setPosition(Values.MID_SERVO);
        wrist.setPosition(Values.MID_SERVO);
        intakeElbow.setPosition(Values.intakeElbowUp);
        outtakeElbow.setPosition(Values.outtakeElbowDown);

        //make servo go slower
//        intakeSlide1.scaleRange(2000, 600);
//        intakeSlide1.scaleRange(2000, 600);
        intakeSlide1.setPosition(Values.slide1in);
        intakeSlide2.setPosition(Values.slide2in);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            double motorSpeed;
            double botHeading;
            if (currentGamepad1.options && !previousGamepad1.options) {
//                posX = drive.pose.position.x;
//                posY = drive.pose.position.y;
//               drive.pose =  new Pose2d(posX,posY,Math.toRadians(90));
                imu.resetYaw();
            }

            if (currentGamepad2.left_stick_y != 0 || currentGamepad2.left_stick_x != 0 || currentGamepad2.right_stick_x != 0) {
                //make it slower
                motorSpeed = Values.speedReducer;
                double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad2.left_stick_x;
                double rx = gamepad2.right_stick_x;
                //double botHeading = drive.pose.heading.toDouble();
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator * motorSpeed;
                double backLeftPower = (rotY - rotX + rx) / denominator * motorSpeed;
                double frontRightPower = (rotY - rotX - rx) / denominator * motorSpeed;
                double backRightPower = (rotY + rotX - rx) / denominator * motorSpeed;
                leftFrontDrive.setPower(frontLeftPower);
                leftBackDrive.setPower(backLeftPower);
                rightFrontDrive.setPower(frontRightPower);
                rightBackDrive.setPower(backRightPower);

            } else {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;
                //double botHeading = drive.pose.heading.toDouble();
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;
                leftFrontDrive.setPower(frontLeftPower);
                leftBackDrive.setPower(backLeftPower);
                rightFrontDrive.setPower(frontRightPower);
                rightBackDrive.setPower(backRightPower);
            }
            if (currentGamepad1.share && previousGamepad1.share) {
                while (botHeading != 0) {
                    leftFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightFrontDrive.setPower(-1);
                    rightBackDrive.setPower(-1);
                  botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
            }

            //slide motors
            if (currentGamepad1.left_trigger != 0 || currentGamepad1.right_trigger != 0) {

                if (currentGamepad1.right_trigger > 0) {
                    slidePosition[0] += (int) (20 * currentGamepad1.right_trigger);
                }
                // Move Slide Down
                if (currentGamepad1.left_trigger > 0) {
                    slidePosition[0] -= (int) (20 * currentGamepad1.left_trigger);
                }
                //straight up or down
                else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    slidePosition[0] = Values.slideMax;
                } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    slidePosition[0] = 0;
                }

                // Ensure slides stay within bounds
                if (slidePosition[0] < 0) {
                    slidePosition[0] = 0;
                }

                if (slidePosition[0] > Values.slideMax) {
                    slidePosition[0] = Values.slideMax;
                }
                moveSlides(slidePosition[0], Values.velocity);
            }
            if (currentGamepad1.circle && previousGamepad1.circle) {
                if (!slidesIsUp) {
                    slidePosition[0] = Values.slideMax;
                    slidesIsUp = true;
                } else if (slidesIsUp) {
                    slidePosition[0] = 0;
                    slidesIsUp = false;
                }
            }

            //all servo stuff


            // wtf happened here
            sethControlThing = outtakeElbow.getPosition();
            //trigger controls gp2
            while (gamepad2.left_trigger > 0) {
                sethControlThing += 0.001;
                if (sethControlThing > .85){
                    sethControlThing = .85;
                }
                outtakeElbow.setPosition(sethControlThing);
            }
            while (gamepad2.right_trigger > 0) {
                sethControlThing -= 0.001;
                if (sethControlThing < 0){
                    sethControlThing = 0;
                }
                outtakeElbow.setPosition(sethControlThing);
            }

            //horizontal slides in N out
            {
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                        slidesOut();
                } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                        slidesIn();
                }
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                slideServo(true);
            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                slideServo(false);
            }

            //intake open N close
            //don know y its kinda jank
            if (currentGamepad2.circle && !previousGamepad2.circle) {
                if (!elbowIsDown) {
                    intakeElbow.setPosition(Values.intakeElbowWait);
                    intakeClaw.setPosition(Values.intakeClawOpen);
                    wrist.setPosition(Values.wristDown);
                    outtakeClaw.setPosition(Values.outakeclawOpen);
                    outtakeElbow.setPosition(Values.outtakeElbowDown);
                    slidePosition[0] = 0;
                    moveSlides(slidePosition[0], Values.velocity);
                    elbowIsDown = true;
                } else if (elbowIsDown == true) {
                    intakeElbow.setPosition(Values.intakeElbowDown);
                    sleep(500);
                    intakeClaw.setPosition(Values.intakeclawClose);
                    sleep(400);
                    wrist.setPosition(Values.wristUp);
                    clawPivot.setPosition(Values.MID_SERVO);
                    intakeElbow.setPosition(Values.intakeElbowUp);
                    slidesTransfer();
                    //slidesIn();
                    sleep(1200);
                    outtakeClaw.setPosition(Values.outtakeClawClose);
                    sleep(400);
                    intakeClaw.setPosition(Values.intakeClawOpen);
                    sleep(200);
                    intakeElbow.setPosition(Values.intakeElbowWait);
                    outtakeElbow.setPosition(Values.outtakeElbowUp);
                    elbowIsDown = false;
                }
            }
                if (currentGamepad2.cross && !previousGamepad2.cross) {
                    if (elbowIsDown) {
                        intakeElbow.setPosition(Values.intakeElbowDown);
                        sleep(500);
                        intakeClaw.setPosition(Values.intakeclawClose);
                        sleep(200);
                        wrist.setPosition(Values.wristUp);
                        intakeElbow.setPosition(Values.intakeElbowUp);
                        slidesIn();
                    } else if (!elbowIsDown) {
                        intakeElbow.setPosition(Values.intakeElbowWait);
                        wrist.setPosition(Values.wristDown);
                        sleep(800);
                        intakeClaw.setPosition(Values.intakeClawOpen);
                    }
                }

                //outtake
                if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                    if (!outtakeIsDown) {
                        outtakeClaw.setPosition(Values.outakeclawOpen);
                        sleep(200);
                        outtakeElbow.setPosition(Values.outtakeElbowDown);
                        outtakeIsDown = true;
                    } else if (outtakeIsDown) {
                        outtakeClaw.setPosition(Values.outtakeClawClose);
                        sleep(200);
                        outtakeElbow.setPosition(Values.outtakeElbowUp);
                        outtakeIsDown = false;
                    }
                }
                if (currentGamepad2.square && !previousGamepad1.square) {
                    if (!outtakeIsFlat) {
                        outtakeClaw.setPosition(Values.outakeclawOpen);
                        sleep(200);
                        outtakeElbow.setPosition(Values.outtakeElbowFlat);
                        outtakeIsFlat = true;
                    } else if (outtakeIsFlat) {
                        outtakeClaw.setPosition(Values.outtakeClawClose);
                        sleep(200);
                        outtakeElbow.setPosition(Values.outtakeElbowUp);
                        outtakeIsFlat = false;
                    }
                }
                //pivot!
                if (currentGamepad2.dpad_right) {
                    clawPivot.setPosition(clawPivot.getPosition() - 0.004);
                } else if (currentGamepad2.dpad_left) {
                    clawPivot.setPosition(clawPivot.getPosition() + 0.004);
                } else if (currentGamepad2.share) {
                    clawPivot.setPosition(Values.MID_SERVO);
                }


                //Send telemetry message to signify robot running;
                telemetry.addData("Intake Claw (circle)", "%.02f", intakeClaw.getPosition());
                telemetry.addData("Intake pitch angle (square)", "%.02f", wrist.getPosition());
                telemetry.addData("Intake big rotate (cross)", "%.02f", intakeElbow.getPosition());
                telemetry.addData("outake, triangle, elbow claw", "%.02f, %.02f", outtakeElbow.getPosition(), outtakeClaw.getPosition());
                telemetry.addData("slides servos (dpad up, or leftNright bumpy)", "%.02f, %.02f", intakeSlide1.getPosition(), intakeSlide2.getPosition());
                telemetry.addData("motor position", outtakeSlide1.getCurrentPosition());
                telemetry.update();
            }

        }

    private void slidesOut () {
        intakeSlide1.setPosition(Values.slide1out);
        intakeSlide2.setPosition(Values.slide2out);
    }
    private void slidesIn () {
        intakeSlide1.setPosition(Values.slide1in);
        intakeSlide2.setPosition(Values.slide2in);
    }
    private void slidesTransfer () {
        intakeSlide1.setPosition(Values.slide1wait);
        intakeSlide2.setPosition(Values.slide2wait);
    }
    private void moveSlides ( int distance, double velocity){
        outtakeSlide1.setTargetPosition(distance);
        outtakeSlide2.setTargetPosition(distance);

        outtakeSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeSlide1.setPower(velocity);
        outtakeSlide2.setPower(velocity);
    }
    private void slideServo (boolean goingOut){
        if (goingOut) {
            intakeSlide2.setPosition(intakeSlide2.getPosition() + 0.05);
            intakeSlide1.setPosition(intakeSlide1.getPosition() - 0.05);
        } else {
            intakeSlide2.setPosition(intakeSlide2.getPosition() - 0.05);
            intakeSlide1.setPosition(intakeSlide1.getPosition() + 0.05);
        }
    }
}

