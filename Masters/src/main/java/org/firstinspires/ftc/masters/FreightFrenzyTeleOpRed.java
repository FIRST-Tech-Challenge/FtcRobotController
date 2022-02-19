package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.masters.FreightFrenzyConstants.region1;
import static org.firstinspires.ftc.masters.FreightFrenzyConstants.region2;

@TeleOp(name="freight Frenzy Red", group = "competition")
public class FreightFrenzyTeleOpRed extends LinearOpMode {


    RobotClass robot;

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
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;

    // operational constants
    double joyScale = 1;
    double motorMax = 0.99; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    int linearSlideTolerance = 5;

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

    linearSlideTargets linearSlideTarget = linearSlideTargets.BASE;
    linearSlidePositions linearSlidePos = linearSlidePositions.BASE;
    boolean carouselOn = false; //Outside of loop()
    boolean carouselPushed = false;
    int encoderPos=0;
    double velocity=0;
    boolean start=false;
    boolean startCarousel = true;
    ElapsedTime elapsedTimeCarousel;
    double vel2Max=0;
    double vel1Max=0;



    @Override
    public void runOpMode() {
        /*
            Buttons Being Used Gamepad 2
            A - Intake
            B - Intake
            X - Intake
            Y - Toggle Carousel
            Dpad up - Raise
            Dpad down - Raise
            Dpad Left - Raise
            Dpad Right - Raise
            Left Trigger - Dump
            Right Trigger - Dump
        */
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new RobotClass(hardwareMap, telemetry, this);

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

        distanceSensorIntake = (DistanceSensor) hardwareMap.get("intakeSensor");
        distanceSensorTop = (DistanceSensor) hardwareMap.get("topDistanceSensor");

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
        carouselMotor.setVelocityPIDFCoefficients(10,0,0.01,14);


        robot.lightSet();
        robot.redLED.setState(true);
        robot.redLED2.setState(true);

        boolean intakeOn = false;
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elapsedTimeCarousel = new ElapsedTime();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("encode",  + linearSlideMotor.getCurrentPosition());
            telemetry.update();

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

            if(gamepad2.a) {
                robot.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                intakeMotor.setPower(-.8);
                intakeOn = true;

            } else if (gamepad2.b) {
                intakeMotor.setPower(.8);
                intakeOn = false;
            } else if (gamepad2.x){
                intakeOn = false;
                intakeMotor.setPower(0);
            }

            if (gamepad2.right_trigger >= .35) {
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                linearSlideTarget = linearSlideTargets.BASE;
                linearSlideMotor.setTargetPosition(20);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(-.4);
            }

            if (gamepad2.dpad_up) {
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

            if (gamepad2.dpad_right) {
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

            if (gamepad2.dpad_down) {
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

            if (gamepad2.dpad_left) {
                //capping
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_CAP);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);
            }

            if (gamepad2.left_trigger >= .35) {
                if (linearSlideMotor.getCurrentPosition() >= 500) {
                    leftFrontMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    leftRearMotor.setPower(0);
                    rightRearMotor.setPower(0);
//                    dump
                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
                    sleep(800);

                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                    robot.pause(400);//200 enough for middle
                    robot.greenLED.setState(false);
                    robot.greenLED.setState(false);

                    robot.redLED.setState(true);
                    robot.redLED2.setState(true);
                    linearSlideTarget = linearSlideTargets.BASE;
                    linearSlideMotor.setTargetPosition(20);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlideMotor.setPower(-.4);//-.4
                    //Wayne trying to go backwards for a while
                    robot.forward(0.5,-.5);
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

            rotateCarousel();

//            double distance = distanceSensorIntake.getDistance(DistanceUnit.CM);
//            telemetry.addData("distance", distance);
//            telemetry.update();
//            if (distance < 7) {
//                intakeMotor.setPower(0);
//                intakeOn = false;
//            }


            double intakeDistance = distanceSensorIntake.getDistance(DistanceUnit.CM);

                if (intakeOn && (intakeDistance<10 || distanceSensorTop.getDistance(DistanceUnit.CM) < 13.5) ) {

                    robot.pauseButInSecondsForThePlebeians(.1);
                    intakeMotor.setPower(0);
                    robot.redLED.setState(false);
                    robot.greenLED.setState(true);
                    robot.redLED2.setState(false);
                    robot.greenLED2.setState(true);
                    intakeOn= false;
                    //robot.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                }

                if (distanceSensorTop.getDistance(DistanceUnit.CM)>13.5){
                    robot.redLED.setState(true);
                    robot.greenLED.setState(false);
                    robot.redLED2.setState(true);
                    robot.greenLED2.setState(false);
                }


//            if (gamepad2.dpad_right) {
//                //Thing that will be doing to find the thing
//                robot.distanceSensorStuff();
//            }

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

}