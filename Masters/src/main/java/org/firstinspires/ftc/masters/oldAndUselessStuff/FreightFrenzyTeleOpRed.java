package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="freight Frenzy Red", group = "competition")
public class FreightFrenzyTeleOpRed extends LinearOpMode {


    RobotClass robot;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor intakeMotor = null;
    DcMotor linearSlideMotor = null;

    Servo dumpServo = null;

    DcMotor carouselMotor = null;
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

    @Override
    public void runOpMode() {
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
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlide");

        dumpServo = hardwareMap.servo.get("dumpServo");
        dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

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

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean intakeOn = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //intakeMotor.setPower(0);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
                if(intakeMotor.getPower() != 0)
                    intakeMotor.setPower(0);
                else
                    intakeMotor.setPower(-.8);
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

            if (gamepad2.dpad_left) {
//                Middle scoring
                dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
                linearSlideTarget = linearSlideTargets.MIDDLE;
                intakeMotor.setPower(0);
                intakeOn = false;
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
                    sleep(1200);

                    dumpServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
                    linearSlideTarget = linearSlideTargets.BASE;
                    linearSlideMotor.setTargetPosition(0);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlideMotor.setPower(-.4);//-.4
                    //Wayne trying to go backwards for a while
                    robot.forward(0.5,-.5);
                }
            }

            linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (linearSlideTarget == linearSlideTargets.BASE) {
                if (linearSlideMotor.getCurrentPosition() <= 10) {
                    linearSlideMotor.setPower(0);
                    linearSlidePos = linearSlidePositions.BASE;
                    intakeOn = true;
                }
            }

            rotateCarousel();

        }
    }
    protected void rotateCarousel(){
        if(gamepad2.y && !carouselOn) {
            if(carouselMotor.getPower() != 0) carouselMotor.setPower(0);
            else carouselMotor.setPower(.6);
            carouselOn = true;
        } else if(!gamepad2.y) carouselOn = false;
    }
}