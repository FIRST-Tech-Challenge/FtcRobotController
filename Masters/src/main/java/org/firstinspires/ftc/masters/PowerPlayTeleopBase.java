package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Power Play FPA Build Day", group = "competition")
public class PowerPlayTeleopBase extends LinearOpMode {


    //RobotClass robot;
   // SampleMecanumDriveCancelable drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    //DcMotor intakeMotor = null;
    DcMotor linearSlideMotor = null;

    Servo clawServo = null;
    Servo armServo = null;



    double maxPowerConstraint = 0.75;

    //Fix values
    public static int SLIDE_HIGH = 1700;
    public static int SLIDE_MIDDLE = 1300;
    public static int SLIDE_BOTTOM = 800;
    public static int SLIDE_BASE = 400;


    public enum linearSlidePositions {
        HIGH,
        MIDDLE,
        BOTTOM,
        BASE,
        CONE_1,
        CONE_2,
        CONE_3,
        CONE_4,
        CONE_5
    }

    linearSlidePositions linearSlideTarget = linearSlidePositions.BASE;

    //Fix values
    protected final double clawServoOpen = 0.79;
    protected final double clawServoClosed = 0.05;
    protected final double armServoBottom = 0.79;
    protected final double armServoTop = 0.05;
    protected double clawServoPos = clawServoOpen;

    int strafeConstant=1;

    @Override
    public void runOpMode() {
        /*
            Controls
            --------

        */
        telemetry.addData("Status", "Initialized");
       // telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlide");

        clawServo = hardwareMap.servo.get("clawServo");
        armServo = hardwareMap.servo.get("armServo");
        clawServo.setPosition(clawServoOpen);

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        armServo.setPosition(armServoBottom);

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("encode",  + linearSlideMotor.getCurrentPosition());
            double y = 0;
            double x = 0;
            double rx = 0;
            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);

            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }
                

            double leftFrontPower = y + strafeConstant* x + rx;
            double leftRearPower = y - strafeConstant* x + rx;
            double rightFrontPower = y - strafeConstant* x - rx;
            double rightRearPower = y + strafeConstant*x - rx;



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

            if (gamepad1.a) {
                maxPowerConstraint = 1;
            }

            if (gamepad1.x) {
                maxPowerConstraint = 0.75;
            }

            if (gamepad1.b) {
                maxPowerConstraint = 0.25;
            }

            if (gamepad2.a) {
                clawServo.setPosition(clawServoClosed);
            }

            if (gamepad2.b) {
                clawServo.setPosition(clawServoOpen);
            }



            if (gamepad2.dpad_up) {
                linearSlideTarget = linearSlidePositions.HIGH;
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setTargetPosition(SLIDE_HIGH);
                linearSlideMotor.setPower(-.4);
            }

            if (gamepad2.dpad_right) {
                clawServo.setPosition(clawServoClosed);
                linearSlideTarget = linearSlidePositions.MIDDLE;
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setTargetPosition(SLIDE_MIDDLE);
                linearSlideMotor.setPower(.9);
            }

            if (gamepad2.dpad_down) {
                clawServo.setPosition(clawServoClosed);
                linearSlideTarget = linearSlidePositions.BOTTOM;
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setTargetPosition(SLIDE_BOTTOM);
                linearSlideMotor.setPower(.9);
            }

            if (gamepad2.dpad_left) {
                clawServo.setPosition(clawServoClosed);
                linearSlideTarget = linearSlidePositions.BASE;
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setTargetPosition(SLIDE_BASE);
                linearSlideMotor.setPower(.9);
            }

            if (Math.abs(gamepad2.left_stick_y) < 0.2) {
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linearSlideMotor.setPower(gamepad2.left_stick_y/2);
            }

            if (gamepad2.right_stick_y > 0.2) {
                armServo.setPosition(armServoTop);
            } else if (gamepad2.right_stick_y < -0.2) {
                armServo.setPosition(armServoBottom);
            } else {
                armServo.setPosition(armServo.getPosition());
            }

//            
//            if (gamepad2.right_bumper){
//                clawServoPos= clawServoOpen;
//                armServo.setPosition(clawServoPos);
//            }

//            if (gamepad2.right_trigger >= .35) {
//                clawServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
//                linearSlideTarget = linearSlidePositions.BASE;
//                linearSlideMotor.setTargetPosition(20);
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearSlideMotor.setPower(-.4);
//            }

//            if (gamepad2.dpad_up) {
////                Top scoring
//                intakeMotor.setPower(-0.8);
//                clawServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
//                linearSlideTarget = linearSlidePositions.TOP;
//
//                intakeOn = false;
//                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearSlideMotor.setPower(.9);
//                intakeMotor.setPower(0);
//            }
//
//            if (gamepad2.dpad_right) {
////                Middle scoring
//                intakeMotor.setPower(-0.8);
//                clawServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
//                linearSlideTarget = linearSlidePositions.MIDDLE;
//
//                intakeOn = false;
//                //robot.pause(400);
//                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearSlideMotor.setPower(.9);
//                intakeMotor.setPower(0);
//            }
//
//            if (gamepad2.dpad_down) {
////                Low scoring
//                intakeMotor.setPower(-0.8);
//                clawServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
//                linearSlideTarget = linearSlidePositions.BOTTOM;
//
//                intakeOn = false;
//                linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                linearSlideMotor.setPower(.8);//.4
//                intakeMotor.setPower(0);
//            }

            telemetry.update();
        }
    }
}