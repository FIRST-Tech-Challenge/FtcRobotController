package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="slide position", group = "test")
public class armSlidePositioning extends LinearOpMode {


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
    DcMotor frontSlide = null;
    DcMotor slideOtherer = null;


    Servo clawServo = null;
    DcMotor armMotor = null;
    Servo tippingServo = null;

    double maxPowerConstraint = 0.75;

    //Fix values
    public static int SLIDE_HIGH = 1150;
    public static int SLIDE_MIDDLE = 550;
    public static int SLIDE_BOTTOM = 0;

    //Fix values
    protected final double clawServoOpen = 0.75;
    protected final double clawServoClosed = 0.99;
    protected final int armMotorBottom = 0;
    public static int armMotorTop = 600;
    public static int armMotorMid = 450;
    public static int armMotorBottomJunction = 330;

    public final double tipCenter = 0.77;
    public final double tipBack = 0.6;
    public final double tipFront =0.99;


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
        frontSlide = hardwareMap.dcMotor.get("frontSlide");
        slideOtherer = hardwareMap.dcMotor.get("slideOtherer");

        clawServo = hardwareMap.servo.get("clawServo");
        armMotor = hardwareMap.dcMotor.get("armServo");
        tippingServo = hardwareMap.servo.get("tippingServo");

        // Set the drive motor direction:
       // leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideOtherer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideOtherer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideOtherer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
//        armMotor.setPosition(.15);
        clawServo.setPosition(clawServoClosed);
        tippingServo.setPosition(tipFront);
        boolean moveArm = false;
        boolean openClaw = true;

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());
            telemetry.addData("arm encoder", armMotor.getCurrentPosition());

            if (gamepad2.a){
                clawServo.setPosition(clawServoClosed);
            }

            if (gamepad2.y){
                clawServo.setPosition(clawServoOpen);
            }

            if (gamepad2.b){
                tippingServo.setPosition(tipFront);
            }


            if (gamepad2.dpad_up) {
                moveArm = false;
                clawServo.setPosition(clawServoClosed);
                armMotor.setTargetPosition(armMotorMid);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                linearSlideMotor.setTargetPosition(SLIDE_HIGH);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlide.setTargetPosition(SLIDE_HIGH);
                frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOtherer.setTargetPosition(SLIDE_HIGH);
                slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.6);
                frontSlide.setPower(.6);
                slideOtherer.setPower(.6);
            }

            if (gamepad2.dpad_right) {
                moveArm = false;
                clawServo.setPosition(clawServoClosed);
                armMotor.setTargetPosition(armMotorMid);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                linearSlideMotor.setTargetPosition(SLIDE_MIDDLE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOtherer.setTargetPosition(SLIDE_MIDDLE);
                slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlide.setTargetPosition(SLIDE_MIDDLE);
                frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlide.setPower(0.6);
                linearSlideMotor.setPower(.6);
                slideOtherer.setPower(.6);
            }

            if (gamepad2.dpad_down) {
                moveArm = false;
                clawServo.setPosition(clawServoClosed);
                armMotor.setTargetPosition(armMotorBottomJunction);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                tippingServo.setPosition(tipFront);
                linearSlideMotor.setTargetPosition(SLIDE_BOTTOM);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlide.setTargetPosition(SLIDE_BOTTOM);
                frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOtherer.setTargetPosition(SLIDE_BOTTOM);
                slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.6);
                frontSlide.setPower(0.6);
                slideOtherer.setPower(0.6);
            }

            if (gamepad2.dpad_left) {
                moveArm = false;
                clawServo.setPosition(clawServoClosed);
                linearSlideMotor.setTargetPosition(SLIDE_BOTTOM);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontSlide.setTargetPosition(SLIDE_BOTTOM);
                frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideOtherer.setTargetPosition(SLIDE_BOTTOM);
                slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(-.4);
                linearSlideMotor.setPower(-0.4);
                slideOtherer.setPower(-0.4);
                armMotor.setTargetPosition(armMotorBottom);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
            }

            if (gamepad2.left_bumper) {
                armMotor.setTargetPosition(armMotorTop);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
            }

            if (gamepad2.left_trigger>0.1){
                slideOtherer.setPower(0.3);
            }


            if (gamepad2.right_stick_y > 0.6) {
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                armMotor.setTargetPosition(armMotorBottom);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                moveArm= false;
            } else if (gamepad2.right_stick_y < -0.6) {
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                armMotor.setTargetPosition(armMotorMid);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
                moveArm=false;
            }


            if (gamepad2.left_stick_y>0.6){
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveArm = true;
                armMotor.setPower(-0.3);
            }
            else if (gamepad2.left_stick_y<-0.6){
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                moveArm = true;
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(0.4);
            }else if (moveArm) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
            }

            telemetry.update();
        }
    }
}