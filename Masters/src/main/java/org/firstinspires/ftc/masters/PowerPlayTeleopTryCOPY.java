package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
//@TeleOp(name="Power Play TeleOp DO NOT USE copy", group = "test")
public class PowerPlayTeleopTryCOPY extends LinearOpMode {


    //RobotClass robot;
   // SampleMecanumDriveCancelable drive;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController armController, liftController;

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

    private final double ticks_in_degree = 1425/360;


    Servo clawServo = null;
    DcMotor armMotor = null;
    Servo tippingServo = null;

    double maxPowerConstraint = 0.75;

    public static double f_lift=0.02;
    public static double f_arm = 0.4;

    //Fix values
    public static int SLIDE_HIGH = 1000;
    public static int SLIDE_MIDDLE = 400;
    public static int SLIDE_BOTTOM = 0;

    public static double d = 0.009;
    public static double p =0.035;
    public static double d_arm=0.0009;
    public static double p_arm=0.05;

    //Fix values
    protected final double clawServoOpen = 0.75;
    protected final double clawServoClosed = 0.99;
    protected final int ARM_BOTTOM = 0;
    public final int ARM_MID_TOP = 350;
    public final int ARM_BOTTOM_JUNCTION = 300;
    protected final int ARM_BACK = 350+1400/4;

    protected final double TIP_CENTER = 0.77;
    protected final double TIP_BACK = 0.6;
    protected final double TIP_FRONT =0.99;


    int strafeConstant=1;

    STATE currentState= STATE.ZERO, previousState = STATE.ZERO;


    int armSelection =0, slideSelection =0;

    int armTarget =0, slideTarget =0;

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

        linearSlideMotor = hardwareMap.get(DcMotorEx.class,"linearSlide");
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontSlide = hardwareMap.get(DcMotorEx.class,"frontSlide");
        frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideOtherer = hardwareMap.get(DcMotorEx.class, "slideOtherer");
        slideOtherer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor = hardwareMap.get(DcMotorEx.class, "armServo");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideOtherer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftController = new PIDController(p, 0, d);
        armController = new PIDController(p_arm, 0, d_arm);

        // Wait for the game to start (driver presses PLAY)

        clawServo.setPosition(clawServoClosed);
        tippingServo.setPosition(TIP_CENTER);
        boolean moveArm = false;
        boolean openClaw = true;


        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());
            telemetry.addData("slideTarget", slideTarget);
//            double y = 0;
//            double x = 0;
//            double rx = 0;
//            telemetry.addData("left y", gamepad1.left_stick_y);
//            telemetry.addData("left x", gamepad1.left_stick_x);
//            telemetry.addData("right x", gamepad1.right_stick_x);
//
//            y = -gamepad1.left_stick_y;
//            x = gamepad1.left_stick_x;
//            rx = gamepad1.right_stick_x;
//            if (Math.abs(y) < 0.2) {
//                y = 0;
//            }
//            if (Math.abs(x) < 0.2) {
//                x = 0;
//            }
//
//            double leftFrontPower = y + strafeConstant* x + rx;
//            double leftRearPower = y - strafeConstant* x + rx;
//            double rightFrontPower = y - strafeConstant* x - rx;
//            double rightRearPower = y + strafeConstant*x - rx;
//
//            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
//
//                double max;
//                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
//                max = Math.max(max, Math.abs(rightFrontPower));
//                max = Math.max(max, Math.abs(rightRearPower));
//
//                leftFrontPower /= max;
//                leftRearPower /= max;
//                rightFrontPower /= max;
//                rightRearPower /= max;
//            }
//
//            leftFrontMotor.setPower(leftFrontPower * maxPowerConstraint);
//            leftRearMotor.setPower(leftRearPower * maxPowerConstraint);
//            rightFrontMotor.setPower(rightFrontPower * maxPowerConstraint);
//            rightRearMotor.setPower(rightRearPower * maxPowerConstraint);
//
//            telemetry.addData("left front power", leftFrontPower);
//            telemetry.addData("left back power", leftRearPower);
//            telemetry.addData("right front power", rightFrontPower);
//            telemetry.addData("right back power", rightRearPower);

            if (gamepad1.a) {
                maxPowerConstraint = 1;
            }

            if (gamepad1.x) {
                maxPowerConstraint = 0.75;
            }

            if (gamepad1.b) {
                maxPowerConstraint = 0.25;
            }
//
            if (gamepad2.a) {
                clawServo.setPosition(clawServoClosed);
                openClaw = false;
            }
//
            if (gamepad2.b) {
                clawServo.setPosition(clawServoOpen);
                openClaw = true;
            }

            if (gamepad2.dpad_up && gamepad2.left_trigger>0.1){
                previousState = currentState;
                currentState = STATE.HIGH;
                armSelection = ARM_BACK;
                slideSelection= SLIDE_HIGH;

                if (previousState == STATE.ZERO){
                    armTarget= armSelection;
                } else {
                    slideTarget= slideSelection;
                    armTarget = armSelection;
                }
            }
            else if (gamepad2.dpad_up){

                previousState = currentState;
                currentState = STATE.HIGH;
                armSelection = ARM_MID_TOP;
                slideSelection= SLIDE_HIGH;

                if (previousState == STATE.ZERO){
                    armTarget= ARM_MID_TOP;
                } else {
                    slideTarget= slideSelection;
                    armTarget = armSelection;
                }

            }

            if (gamepad2.dpad_right && gamepad2.left_trigger>0.1){
                previousState = currentState;
                currentState = STATE.MID;
                slideSelection = SLIDE_MIDDLE;
                armSelection = ARM_BACK;
                if (previousState == STATE.ZERO){
                    armTarget = armSelection;
                } else {
                    armTarget = armSelection;
                    slideTarget = slideSelection;
                }
            }
           else if (gamepad2.dpad_right) {
                previousState = currentState;
                currentState = STATE.MID;
                slideSelection = SLIDE_MIDDLE;
                armSelection = ARM_MID_TOP;
                if (previousState == STATE.ZERO){
                    armTarget = ARM_MID_TOP;
                } else {
                    armTarget = ARM_MID_TOP;
                    slideTarget = slideSelection;
                }
            }

            if (gamepad2.dpad_down) {

                previousState = currentState;
                currentState = STATE.BOTTOM;
                armSelection = ARM_BOTTOM_JUNCTION;
                slideSelection= SLIDE_BOTTOM;

                if (previousState == STATE.ZERO){
                    armTarget = ARM_BOTTOM_JUNCTION;
                } else if (previousState== STATE.MID || previousState == STATE.HIGH || previousState == STATE.BACK_HIGH || previousState== STATE.BACK_MID){
                    armTarget = ARM_BOTTOM_JUNCTION;
                    slideTarget = SLIDE_BOTTOM;
                }
            }



            if (gamepad2.dpad_left) {
                previousState = currentState;
                currentState = STATE.ZERO;
                armSelection = ARM_BOTTOM;
                slideSelection = SLIDE_BOTTOM;
                if (previousState == STATE.BOTTOM){
                    armTarget = armSelection;
                } else {
                    slideTarget = slideSelection;
                }
                clawServo.setPosition(clawServoClosed);

            }

            if (currentState == STATE.HIGH || currentState == STATE.MID || currentState == STATE.BACK_HIGH || currentState == STATE.BACK_MID){
                if (slideTarget!=slideSelection){
                    if (armMotor.getCurrentPosition()>100){
                        slideTarget = slideSelection;
                    }
                }
            } else if (currentState == STATE.ZERO && previousState!= STATE.ZERO){
                if (armTarget!=0){
                    if (linearSlideMotor.getCurrentPosition()<100){
                        armTarget=0;
                    }
                }
                if (armMotor.getCurrentPosition()<50){
                    clawServo.setPosition(clawServoOpen);
                    tippingServo.setPosition(TIP_CENTER);
                }
            }
            if (armMotor.getCurrentPosition()>100){
                if (currentState == STATE.BOTTOM || currentState == STATE.MID || currentState == STATE.HIGH){
                    tippingServo.setPosition(TIP_FRONT);
                } else if (currentState== STATE.BACK_HIGH || currentState == STATE.BACK_MID){
                    tippingServo.setPosition(TIP_BACK);
                }
            }



//            switch (stateArm){
//                case START_UP:
//                    if (armMotor.getCurrentPosition()>50){
//                        tippingServo.setPosition(TIP_FRONT);
//                        stateArm = DONE;
//                        stateSlide = STATE_SLIDE.EXTENDING;
//                        slideTarget = slideSelection;
//                       // moveSlideMotors(slideTarget, 0.6);
//                    }
//
//                    break;
//                case START_DOWN:
//                    if (armMotor.getCurrentPosition()<50){
//                        tippingServo.setPosition(TIP_CENTER);
//                    }
//                    if (armMotor.getCurrentPosition()<20){
//                        stateArm = ZERO;
//                        clawServo.setPosition(clawServoOpen);
//                    }
//                    break;
//            }
//
//            switch (stateSlide){
//                case RETRACTING:
//                    if (linearSlideMotor.getCurrentPosition()<50){
//                        clawServo.setPosition(clawServoClosed);
//                        stateArm = START_DOWN;
//                        armTarget = armMotorBottom;
//
////                        armMotor.setTargetPosition(armMotorBottom);
////                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                        armMotor.setPower(0.3);
//                    }
//                    if (linearSlideMotor.getCurrentPosition() < 20) {
//                        stateSlide = STATE_SLIDE.ZERO;
//                        slideSelection = 0;
//                    }
//                    break;
//            }






            if (gamepad2.right_stick_y > 0.6) {
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                armTarget = ARM_BOTTOM;

            } else if (gamepad2.right_stick_y < -0.6) {
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                armTarget = ARM_MID_TOP;
//                armMotor.setTargetPosition(armMotorMid);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.3);
                moveArm=false;
            }


            if (gamepad2.left_stick_y>0.6){
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveArm = true;
               // armMotor.setPower(-0.3);
                if (armTarget>0){
                    armTarget = armTarget -3;
                }
            }
            else if (gamepad2.left_stick_y<-0.6){
                if(openClaw){
                    clawServo.setPosition(clawServoClosed);
                    openClaw = false;
                }
                moveArm = true;
                armTarget = armTarget+3;
               // armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               // armMotor.setPower(0.4);
            }
//            else if (moveArm) {
//                armMotor.setTargetPosition(armMotor.getCurrentPosition());
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.3);
//            }

            moveSlideMotors();
            moveArm();
            telemetry.update();
        }
    }

    protected void moveSlideMotors (){

        int liftPos = linearSlideMotor.getCurrentPosition();
        double pid = liftController.calculate(liftPos, slideTarget);

        double power = pid +f_lift;

        if (slideTarget==0 && liftPos<20){
            linearSlideMotor.setPower(0);
            frontSlide.setPower(0);
            slideOtherer.setPower(0);
        } else {
            linearSlideMotor.setPower(power);
            frontSlide.setPower(power);
            slideOtherer.setPower(power);
        }

        telemetry.addData("slide", linearSlideMotor.getCurrentPosition());
        telemetry.addData("front", frontSlide.getCurrentPosition());
        telemetry.addData("other", slideOtherer.getCurrentPosition());
//
    }

    protected void moveArm (){
        int armPos = armMotor.getCurrentPosition();
        double pid = armController.calculate(armPos, armTarget);

        double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degree))*f_arm;

        double power = pid +ff;
        if (armTarget ==0 && armPos<10){
            armMotor.setPower(0);
        } else {
            armMotor.setPower(power);
        }
    }

    enum STATE{
        ZERO, BOTTOM, MID, HIGH, BACK_MID, BACK_HIGH
    }


}