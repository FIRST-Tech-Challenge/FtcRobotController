package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.FreightFrenzyConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="Freight Frenzy Red 2: Electric Boogaloo", group = "competition")
public class FreightFrenzyTeleOpRedElectricBoogaloo extends LinearOpMode {

    public static int region1 = 1200;
    public static int region2 = 2100;


    //RobotClass robot;
    SampleMecanumDriveCancelable drive;

    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor intakeMotor = null;
    DcMotor linearSlideMotor = null;

    Servo dumpServo = null;
    DistanceSensor distanceSensorIntake, distanceSensorTop;
    DcMotorEx carouselMotor = null;

//    // declare motor speed variables
//    double RF; double LF; double RR; double LR;
//    // declare joystick position variables
//    double X1; double Y1; double X2; double Y2;
//
//    // operational constants
//    double joyScale = 1;
//    double motorMax = 0.99; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
//    int linearSlideTolerance = 5;

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
    linearSlidePositions linearSlidePos = linearSlidePositions.BASE;
    DriveMode currentMode = DriveMode.NORMAL;

    boolean carouselOn = false; //Outside of loop()
    boolean carouselPushed = false;
    int encoderPos=0;
    double velocity=0;
    boolean start=false;
    boolean startCarousel = true;
    ElapsedTime elapsedTimeCarousel;
    double vel2Max=0;
    double vel1Max=0;

    Trajectory pastLineRed;
    Trajectory toHubRed;
    Trajectory toWarehouse;

    @Override
    public void runOpMode() {
        /*
            Buttons Being Used Gamepad 2
            A
            B
            X
            Dpad up
            Dpad down
            Dpad Left
            Left Trigger
            Dpad Right
        */
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new SampleMecanumDriveCancelable(hardwareMap, this, telemetry);

        Pose2d startPose =new Pose2d(new Vector2d(26, -66),Math.toRadians(180));

        drive.setPoseEstimate(startPose);


        telemetry.addData("Status", "Initialized odometry");
        telemetry.update();

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
        dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

        telemetry.addData("Status", "Initialized motors");
        telemetry.update();

        distanceSensorIntake = (DistanceSensor) hardwareMap.get("intakeSensor");
        distanceSensorTop = (DistanceSensor) hardwareMap.get("topDistanceSensor");

        telemetry.addData("Status", "Initialized distance sensors");
        telemetry.update();

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

        telemetry.addData("Status", "Initialized motor modes");
        telemetry.update();

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //drive.lightSet();
        drive.redLED.setState(true);
        drive.redLED2.setState(true);

        boolean intakeOn = false;
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Before trajectories");
        telemetry.update();
        currentMode = DriveMode.NORMAL;


        pastLineRed= drive.trajectoryBuilder(new Pose2d(new Vector2d(26, -66),Math.toRadians(180)))
                .lineTo(new Vector2d(10, -65))
               .splineToSplineHeading(new Pose2d(-12, -45, Math.toRadians(90)), Math.toRadians(90))
              //  .addDisplacementMarker(()->drive.followTrajectoryAsync(toHubRed))
                .build();

//        toHubRed = drive.trajectoryBuilder(pastLineRed.end())
//                .splineToSplineHeading(new Pose2d(-9, -48, Math.toRadians(90)), Math.toRadians(90))
//                .build();



        telemetry.addData("Status", "Initialized trajectory 1");
        telemetry.update();

//        toHubRed = drive.trajectoryBuilder(pastLineRed.end())
//                .splineToSplineHeading(new Pose2d(-11, 48, Math.toRadians(270)), Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(new Vector2d(-12.7, -42), Math.toRadians(90)), Math.toRadians(90) )
//                .build();


        telemetry.addData("Status", "Initialized trajectories");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("encode",  + linearSlideMotor.getCurrentPosition());
            telemetry.update();

           // drive.update();


            switch (currentMode) {
                case NORMAL:

                    double y = 0; //
                    double x = 0;
                    double rx = 0;

                    if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 ) {
                        y = gamepad1.left_stick_y;
                        x = gamepad1.left_stick_x;
                        if (Math.abs(y)<0.2){
                            y=0;
                        }
                        if (Math.abs(x)<0.2){
                            x=0;
                        }

                        rx = gamepad1.right_stick_x;
                    }


                    double leftFrontPower = y + x + rx;
                    double leftRearPower = y - x + rx;
                    double rightFrontPower = y - x - rx;
                    double rightRearPower = y + x - rx;

                    if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                        double max;
                        max = Math.max(leftFrontPower, leftRearPower);
                        max = Math.max(max, rightFrontPower);
                        max = Math.max(max, rightRearPower);

                        leftFrontPower /= max;
                        leftRearPower /= max;
                        rightFrontPower /= max;
                        rightRearPower /= max;
                    }

                    leftFrontMotor.setPower(leftFrontPower*maxPowerConstraint);
                    leftRearMotor.setPower(leftRearPower*maxPowerConstraint);
                    rightFrontMotor.setPower(rightFrontPower*maxPowerConstraint);
                    rightRearMotor.setPower(rightRearPower*maxPowerConstraint);

                    if(gamepad1.a) {
                        maxPowerConstraint = 1;
                    }

                    if(gamepad1.x){
                        maxPowerConstraint = 0.75;
                    }

                    if (gamepad1.b) {
                        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
                        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
                    }

                    if (gamepad1.y) {
                        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
                        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
                        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
                        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
                    }

                    if (gamepad1.right_trigger > .35) {
                        currentMode = DriveMode.TO_HUB;
                        boolean foundWhite = toLineTeleop(1.5);
                        if (foundWhite) {
                            drive.followTrajectoryAsync(pastLineRed);
                        }
                    }
                    break;

                case TO_HUB:

                    if (!drive.isBusy()) {
                        currentMode = DriveMode.NORMAL;
                    }

                    break;



                case TO_SHARED_HUB:
                    break;

                case TO_WAREHOUSE:
                    break;
            }

            drive.update();

            if (gamepad1.right_bumper) {
                currentMode = DriveMode.TO_HUB;
            }

            if(gamepad2.a) {
                drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                intakeMotor.setPower(-1);
                intakeOn = true;

            } else if (gamepad2.b) {
                intakeMotor.setPower(.8);
                intakeOn = false;
            } else if (gamepad2.x){
                intakeOn = false;
                intakeMotor.setPower(0);
            }

            if (gamepad2.dpad_up) {
//                Top scoring
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.TOP;
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.9);
            }

            if (gamepad2.dpad_right) {
//                Middle scoring
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.MIDDLE;
                intakeMotor.setPower(0);
                intakeOn = false;
                drive.pause(400);
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.9);
            }

            if (gamepad2.dpad_down) {
//                Low scoring
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.BOTTOM;
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);//.4
            }

            if (gamepad2.left_trigger >= .35) {
                if (linearSlideMotor.getCurrentPosition() >= 500) {
//                    dump
                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
                    sleep(600);

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
                    drive.forward(0.5,-.5);
                }
            }

            linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            if (linearSlideTarget == linearSlideTargets.BASE) {
//                if (linearSlideMotor.getCurrentPosition() <= 10) {
//                    linearSlideMotor.setPower(0);
//                    linearSlidePos = linearSlidePositions.BASE;
//                    intakeOn = true;
//                }
//            }



            rotateCarousel();

            double intakeDistance = distanceSensorIntake.getDistance(DistanceUnit.CM);

            if (intakeOn && (intakeDistance<10 || distanceSensorTop.getDistance(DistanceUnit.CM) < 13.5) ) {

                drive.pauseButInSecondsForThePlebeians(.1);
                intakeMotor.setPower(0);
                drive.redLED.setState(false);
                drive.greenLED.setState(true);
                drive.redLED2.setState(false);
                drive.greenLED2.setState(true);
                intakeOn= false;
                //drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
            }

            if (distanceSensorTop.getDistance(DistanceUnit.CM)<13.5){
                drive.redLED.setState(true);
                drive.greenLED.setState(false);
                drive.redLED2.setState(true);
                drive.greenLED2.setState(false);
                intakeMotor.setPower(0);
                intakeOn=false;
            }
            telemetry.addData("intake on", Boolean.toString(intakeOn));


        }
    }

    protected void rotateCarousel(){
        if (gamepad2.y && !carouselPushed)  {
            if (carouselOn){
                carouselOn = false;
                carouselMotor.setVelocity(0);
            } else {
                carouselOn = true;
            }
            carouselPushed= true;

        } else if (!gamepad2.y) {
            carouselPushed = false;
        }

        if (carouselOn) {

            encoderPos = carouselMotor.getCurrentPosition();

            if (encoderPos < region1) {
                velocity = Math.sqrt(2*FreightFrenzyConstants.accelerate1*encoderPos)+FreightFrenzyConstants.startVelocity;
                vel1Max = velocity;
                carouselMotor.setVelocity(velocity);
                telemetry.update();
            } else if (encoderPos >= region1 && encoderPos < region2) {
                velocity = vel1Max + Math.sqrt(2 * FreightFrenzyConstants.accelerate2 * (encoderPos - region1));
                vel2Max = velocity;
                carouselMotor.setVelocity(velocity);
                telemetry.update();
            }
            else if (encoderPos >= region2 && encoderPos < FreightFrenzyConstants.goal) {
                velocity = vel2Max+Math.sqrt(2*FreightFrenzyConstants.accelerate3*(encoderPos-region2));
                carouselMotor.setVelocity(velocity);
                telemetry.update();
            } else if (encoderPos >= FreightFrenzyConstants.goal) {
                carouselOn = false;
                carouselMotor.setVelocity(0);
                carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

        }

    }

    protected boolean toLineTeleop(double time){
        return drive.toLineRedTeleop(time);
    }

}