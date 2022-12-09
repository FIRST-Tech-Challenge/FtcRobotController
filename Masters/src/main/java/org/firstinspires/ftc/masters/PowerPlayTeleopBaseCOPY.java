package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Power Play TeleOp single", group = "competition")
public class PowerPlayTeleopBaseCOPY extends LinearOpMode {


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
    public static int SLIDE_HIGH = 1320;
    public static int SLIDE_MIDDLE = 710;
    public static int SLIDE_BOTTOM = 0;

    //Fix values
    protected final double clawServoOpen = 0.14;
    protected final double clawServoClosed = 0.0;
    protected final double armServoBottom = 0.20;
    protected final double armServoTop = 0.76;
    protected final double armServoMid = 0.57;



    int strafeConstant=1;

    @Override
    public void runOpMode() {
        /*
            Controls
            --------
            Gamepad2
                A: Open claw
                B: Close claw
                Dpad Up: High junction
                Dpad Right: Middle junction
                Dpad Down: Low junction
                Dpad Left: Base
                Left stick Y-axis: Fine linear slide control (Not done yet)
                Right stick: Arm servo control

            Gamepad1
                Steer gud
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
        armServo.setPosition(.15);
        clawServo.setPosition(clawServoOpen);

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

            y = -gamepad2.left_stick_y;
            x = gamepad2.left_stick_x;
            rx = gamepad2.right_stick_x;
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
                clawServo.setPosition(clawServoClosed);
                armServo.setPosition(armServoMid);
                linearSlideMotor.setTargetPosition(SLIDE_HIGH);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.6);
            }

            if (gamepad2.dpad_right) {
                clawServo.setPosition(clawServoClosed);
                armServo.setPosition(armServoMid);
                linearSlideMotor.setTargetPosition(SLIDE_MIDDLE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.6);
            }

            if (gamepad2.dpad_down) {
                clawServo.setPosition(clawServoClosed);
                armServo.setPosition(armServoMid);
                linearSlideMotor.setTargetPosition(SLIDE_BOTTOM);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.6);
            }

            if (gamepad2.dpad_left) {
                clawServo.setPosition(clawServoClosed);
                linearSlideMotor.setTargetPosition(SLIDE_BOTTOM);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(-.4);
                armServo.setPosition(armServoBottom);
            }

            if (gamepad2.left_bumper) {
                armServo.setPosition(armServoTop);
            }

//            if (Math.abs(gamepad2.left_stick_y) < 0.2) {
//                linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                linearSlideMotor.setPower(gamepad2.left_stick_y/2);
//            }

            if (gamepad2.right_stick_y > 0.6) {
                armServo.setPosition(armServoBottom);
            } else if (gamepad2.right_stick_y < -0.6) {
                armServo.setPosition(armServoMid);
            }


            telemetry.update();
        }
    }
}