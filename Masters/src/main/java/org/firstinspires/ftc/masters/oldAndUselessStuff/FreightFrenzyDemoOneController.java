package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.masters.oldAndUselessStuff.FreightFrenzyConstants.region1;
import static org.firstinspires.ftc.masters.oldAndUselessStuff.FreightFrenzyConstants.region2;

@TeleOp(name="FF Demo OC", group = "competition")
public class FreightFrenzyDemoOneController extends LinearOpMode {


    //RobotClass robot;
    SampleMecanumDriveCancelable drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor intakeMotor = null;
    DcMotor linearSlideMotor = null;

    Servo dumpServo = null;
    Servo capServo = null;

    DistanceSensor distanceSensorIntake;

    DcMotorEx carouselMotor = null;
    // declare motor speed variables

    double maxPowerConstraint = 0.75;

    private enum linearSlideTargets {
        TOP,
        MIDDLE,
        BOTTOM,
        BASE
    }

    public enum linearSlidePositions {
        TOP,
        MIDDLE,
        BOTTOM,
        BASE
    }

    public enum DriveMode {
        NORMAL,
        TO_WAREHOUSE,
        TO_HUB,
        TO_SHARED_HUB
    }

    linearSlideTargets linearSlideTarget = linearSlideTargets.BASE;
    DriveMode currentMode = DriveMode.NORMAL;
    protected boolean carouselOn = false; //Outside of loop()
    protected boolean carouselPushed = false;
    protected double encoderPos=0;
    protected double velocity=0;
    protected ElapsedTime elapsedTimeCarousel;
    protected double vel2Max=0;
    protected double vel1Max=0;

    protected final double capServoBottom = 0.79;
    protected final double capServoTop = 0.05;
    protected double capServoPos = capServoBottom;
    protected RevColorSensorV3 intakeColor;

    Trajectory toHub;
    double encoderCorrection =0;
    boolean start = true;

    int strafeConstant=1;

    @Override
    public void runOpMode() {
        /*
            Buttons Being Used Gamepad 2
            A
            B
            X
            Y
            Dpad up
            Dpad down
            Dpad Left
            Dpad Right
            Left Trigger
            Right Trigger
            Left Joystick
        */
        telemetry.addData("Status", "Initialized");
        // telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDriveCancelable(hardwareMap, this, telemetry);

        Pose2d startPose = new Pose2d(new Vector2d(26, -66), Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlide");

        dumpServo = hardwareMap.servo.get("dumpServo");
        capServo = hardwareMap.servo.get("capServo");
        dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

        distanceSensorIntake = (DistanceSensor) hardwareMap.get("intakeSensor");
        //distanceSensorTop = (DistanceSensor) hardwareMap.get("topDistanceSensor");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");

        // Set the drive motor direction:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // These polarities are for the Neverest 20 motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setVelocityPIDFCoefficients(10, 0, 0.01, 14);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.lightSet();
        drive.redLED.setState(true);
        drive.redLED2.setState(true);

        boolean intakeOn = false;
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elapsedTimeCarousel = new ElapsedTime();

        currentMode = DriveMode.NORMAL;

        toHub = getHubTrajectory();

        // Wait for the game to start (driver presses PLAY)
        capServo.setPosition(capServoBottom);

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("encode",  + linearSlideMotor.getCurrentPosition());


            switch (currentMode) {
                case NORMAL:
                    double y = 0;
                    double x = 0;
                    double rx = 0;
                    telemetry.addData("left y", gamepad1.left_stick_y);
                    telemetry.addData("left x", gamepad1.left_stick_x);
                    telemetry.addData("right x", gamepad1.right_stick_x);

                    if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2) {
                        y = gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x;
                        if (Math.abs(y) < 0.2) {
                            y = 0;
                        }
                        if (Math.abs(x) < 0.2) {
                            x = 0;
                        }

                        rx = gamepad1.right_stick_x;
                    }

                    double leftFrontPower = y + strafeConstant * x + rx;
                    double leftRearPower = y - strafeConstant * x + rx;
                    double rightFrontPower = y - strafeConstant * x - rx;
                    double rightRearPower = y + strafeConstant * x - rx;


                    if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                        double max;
                        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                        max = Math.max(max, Math.abs(rightFrontPower));
                        max = Math.max(max, Math.abs(rightRearPower));

                        leftFrontPower /= max;
                        leftRearPower /= max;
                        rightFrontPower /= max;
                        rightRearPower /= max;
                    }

                    leftFrontMotor.setPower(leftFrontPower * maxPowerConstraint);
                    leftRearMotor.setPower(leftRearPower * maxPowerConstraint);
                    rightFrontMotor.setPower(rightFrontPower * maxPowerConstraint);
                    rightRearMotor.setPower(rightRearPower * maxPowerConstraint);

                    telemetry.addData("left front power", leftFrontPower);
                    telemetry.addData("left back power", leftRearPower);
                    telemetry.addData("right front power", rightFrontPower);
                    telemetry.addData("right back power", rightRearPower);

//                    if (gamepad1.a) {
//                        maxPowerConstraint = 1;
//                    }
//
//                    if (gamepad1.x) {
//                        maxPowerConstraint = 0.75;
//                    }
//
//                    if (gamepad1.b) {
//                        maxPowerConstraint = 0.25;
//                    }


                    if (gamepad1.right_trigger > .35) {
                        currentMode = DriveMode.TO_HUB;
                        boolean foundWhite = toLineTeleop(1.5);
                        telemetry.addData("found white", foundWhite);
                        if (foundWhite) {

                            drive.followTrajectoryAsync(toHub);
                        }
                    }
                    break;
                case TO_HUB:

                    if (!drive.isBusy()) {
                        currentMode = DriveMode.NORMAL;
                    }

                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = DriveMode.NORMAL;
                    }

                    //add way to stop trajectory

                    break;
                case TO_SHARED_HUB:
                    break;

                case TO_WAREHOUSE:
                    break;

            }

            drive.update();

            if (Math.abs(gamepad1.left_stick_y) > 0.2 && gamepad1.a) {
                telemetry.addData("left stick 2: ", gamepad1.left_stick_y);
                telemetry.addData("servo pos", capServoPos);
                if (gamepad1.left_stick_y < 0 && gamepad1.a) {
                    if (capServoPos > capServoTop)
                        if (capServoPos < 0.5) {
                            capServoPos = capServoPos - 0.005;
                        } else {
                            capServoPos = capServoPos - 0.005;
                        }
                    capServo.setPosition(capServoPos);
                }
                if (gamepad1.left_stick_y > 0 && gamepad1.a) {
                    if (capServoPos < capServoBottom) {

                        capServoPos = capServoPos + 0.01;

                    }
                    capServo.setPosition(capServoPos);
                }
            }
            telemetry.addData("Cap Servo Pos = ", capServoPos);
//
            if (gamepad1.right_bumper) {
                capServoPos = capServoBottom;
                capServo.setPosition(capServoPos);
            }
//
            if (gamepad1.a && gamepad1.b) {
                drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                intakeMotor.setPower(-.8);
                intakeOn = true;

            } else if (gamepad1.b) {
                intakeMotor.setPower(.8);
                intakeOn = false;
            } else if (gamepad1.x) {
                intakeOn = false;
                intakeMotor.setPower(0);
            }
//
            if (gamepad1.right_trigger >= .35) {
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                linearSlideTarget = linearSlideTargets.BASE;
                linearSlideMotor.setTargetPosition(20);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(-.4);
            }
//
            if (gamepad1.dpad_up) {
//                Top scoring
                intakeMotor.setPower(-0.8);
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.TOP;

                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.9);
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_right) {
//                Middle scoring
                intakeMotor.setPower(-0.8);
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.MIDDLE;

                intakeOn = false;
                //robot.pause(400);
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.9);
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_down) {
//                Low scoring
                intakeMotor.setPower(-0.8);
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.BOTTOM;

                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);//.4
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_left) {
                //capping
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_CAP);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);
            }
//
            if (gamepad1.left_trigger >= .35) {
                if (linearSlideMotor.getCurrentPosition() >= 500) {
                    leftFrontMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    leftRearMotor.setPower(0);
                    rightRearMotor.setPower(0);
//                    dump
                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
                    sleep(800);

                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                    drive.pause(400);//200 enough for middle
                    drive.greenLED.setState(false);
                    drive.greenLED.setState(false);

                    drive.redLED.setState(true);
                    drive.redLED2.setState(true);
                    linearSlideTarget = linearSlideTargets.BASE;
                    linearSlideMotor.setTargetPosition(20);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlideMotor.setPower(-.4);//-.4
                    //Wayne trying to go backwards for a while
                    drive.forward(0.5, -.5);
                }
            }
//
//            linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//            rotateCarousel();

            double intakeDistance = distanceSensorIntake.getDistance(DistanceUnit.CM);

            if (intakeOn && (intakeDistance < 10 || intakeColor.getDistance(DistanceUnit.CM) < 8)) {

                drive.pauseButInSecondsForThePlebeians(.2);

                drive.redLED.setState(false);
                drive.greenLED.setState(true);
                drive.redLED2.setState(false);
                drive.greenLED2.setState(true);
                intakeOn = false;
                // drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                intakeMotor.setPower(0);
            }

//                if (distanceSensorTop.getDistance(DistanceUnit.CM)>13.5){
//                    drive.redLED.setState(true);
//                    drive.greenLED.setState(false);
//                    drive.redLED2.setState(true);
//                    drive.greenLED2.setState(false);
//                }

            telemetry.update();
        }

//    protected void rotateCarousel(){
        if (gamepad1.a && gamepad1.x && !carouselPushed) {
            carouselPushed = true;

            if (carouselOn) {
                carouselOn = false;
                carouselMotor.setVelocity(0);
                carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                carouselOn = true;
                encoderCorrection = carouselMotor.getCurrentPosition();
                carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        } else if (!gamepad1.a && gamepad1.x) {
            carouselPushed = false;
        }

        if (carouselOn) {

            encoderPos = carouselMotor.getCurrentPosition() - encoderCorrection;

            if (encoderPos < region1) {
                velocity = Math.sqrt(2 * FreightFrenzyConstants.accelerate1 * encoderPos) + FreightFrenzyConstants.startVelocity;
                vel1Max = velocity;
                carouselMotor.setVelocity(velocity);
                //telemetry.update();
            } else if (encoderPos >= region1 && encoderPos < region2) {
                velocity = vel1Max + Math.sqrt(2 * FreightFrenzyConstants.accelerate2 * (encoderPos - region1));
                vel2Max = velocity;
                carouselMotor.setVelocity(velocity);
                //telemetry.update();
            } else if (encoderPos >= region2 && encoderPos < FreightFrenzyConstants.goal) {
                velocity = vel2Max + Math.sqrt(2 * FreightFrenzyConstants.accelerate3 * (encoderPos - region2));
                carouselMotor.setVelocity(velocity);
                //telemetry.update();
            } else if (encoderPos >= FreightFrenzyConstants.goal) {
                carouselOn = false;
                carouselMotor.setVelocity(0);
                carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                carouselPushed = false;

            }

        }
    }




        protected boolean toLineTeleop(double time){
            return drive.toLineRedTeleop(time);
        }

    protected Trajectory getHubTrajectory(){
        return  drive.trajectoryBuilder(new Pose2d(new Vector2d(26, -66),Math.toRadians(180)))
                .lineTo(new Vector2d(10, -65))
                .splineToSplineHeading(new Pose2d(-10, -45, Math.toRadians(90)), Math.toRadians(90))
                .build();
    }

}
