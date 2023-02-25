package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BOTTOM;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BOTTOM_JUNCTION;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_CONE_STACK;
import static org.firstinspires.ftc.masters.BadgerConstants.ARM_MID_TOP;
import static org.firstinspires.ftc.masters.BadgerConstants.CLAW_CLOSED;
import static org.firstinspires.ftc.masters.BadgerConstants.CLAW_OPEN;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_BOTTOM;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_CONE_INCREMENT;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_HIGH;
import static org.firstinspires.ftc.masters.BadgerConstants.SLIDE_MIDDLE;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_BACK;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_CENTER;
import static org.firstinspires.ftc.masters.BadgerConstants.TIP_FRONT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name="Power Play TeleOp COLORADO", group = "competition")
public class PowerPlayTeleopColorado extends LinearOpMode {


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    ArmPIDControllerMotionProfile armPIDController;
    LiftPIDController liftPIDController;
    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    //DcMotor intakeMotor = null;
    DcMotorEx linearSlideMotor = null;
    DcMotorEx frontSlide = null;
    DcMotorEx slideOtherer = null;

    private final double MAX_VELOCITY= 435/60*384.5*0.8;
    ProfiledPIDController m_controller;

    public static double ALIGN_SPEED =0.15;

    Servo clawServo = null;
    DcMotorEx armMotor = null;
    Servo tippingServo = null;

    double maxPowerConstraint = 1;

    int strafeConstant=1;

    STATE currentState= STATE.ZERO, previousState = STATE.ZERO;

    int armSelection =0, slideSelection =0;

    int armTarget =0, slideTarget =0;
    boolean set = true;
    boolean trigger = false;

    int stackIncrement=0;

    int liftOffset;
    int armOffset;
    PowerPlayComputerVisionPipelines  CV= null;

    public static double kp=6, ki=0, kd=0.09;
    public static double f_arm = 0.09;
    private final double ticks_in_degree = 2785 / 360;
    private final double ticks_per_seconds = 2785 * 60 /60;


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
        CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
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
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linearSlide");
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        slideOtherer = hardwareMap.get(DcMotorEx.class, "slideOtherer");

        clawServo = hardwareMap.servo.get("clawServo");
        armMotor = hardwareMap.get(DcMotorEx.class,"armServo");
        tippingServo = hardwareMap.servo.get("tippingServo");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
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

        //liftController = new PIDController(p, i, d);
//        armController = new PIDController(p_arm, i_arm, d_arm);
        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(2800, 3000);

        m_controller = new ProfiledPIDController(kp, ki, kd, m_constraints);
        m_controller.setTolerance(3);

        armPIDController = new ArmPIDControllerMotionProfile(armMotor);
        liftPIDController = new LiftPIDController(linearSlideMotor, frontSlide, slideOtherer);

        // Wait for the game to start (driver presses PLAY)

        clawServo.setPosition(CLAW_OPEN);
        tippingServo.setPosition(TIP_CENTER);
        boolean moveArm = false;
        boolean openClaw = true;

        boolean aligning = false;
        boolean yPushed= false;
        CV.setPipeDetectionFront();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());
            telemetry.addData("arm encoder", armMotor.getCurrentPosition());
            double y = 0;
            double x = 0;
            double rx = 0;
            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);

            y = -gamepad1.left_stick_y;
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

//            telemetry.addData("left front power", leftFrontPower);
//            telemetry.addData("left back power", leftRearPower);
//            telemetry.addData("right front power", rightFrontPower);
//            telemetry.addData("right back power", rightRearPower);

            if (gamepad1.a) {
                maxPowerConstraint = 1;
            } else if (gamepad1.x) {
                maxPowerConstraint = 0.75;
            }

            if (gamepad1.y){
                if (!yPushed){
                    aligning = !aligning;
                }

                yPushed = true;

            } else {
                yPushed = false;
            }
            if (aligning){
                if (alignPole()){
                    aligning = false;

                }

            }



            if (gamepad1.left_trigger>0.2){
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(200);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
//
            if (gamepad2.a) {
                clawServo.setPosition(CLAW_CLOSED);

            } else if (gamepad2.b) {
               // tippingServo.setPosition(TIP_CENTER);
                clawServo.setPosition(CLAW_OPEN);

            }

            if (gamepad2.x){
                tippingServo.setPosition(TIP_FRONT);
            }
            if (gamepad2.y){
                tippingServo.setPosition(TIP_BACK);
            }
            if (gamepad2.right_trigger>0.1){
                tippingServo.setPosition(TIP_CENTER);
            }

      if (gamepad2.dpad_up && gamepad2.left_trigger>0.1){
                previousState = currentState;
                currentState = STATE.BACK_HIGH;
                armSelection = ARM_BACK - armOffset;
                slideSelection= SLIDE_HIGH - liftOffset;

                if (previousState == STATE.ZERO){
                    armTarget= armSelection;
                } else {
                    slideTarget= slideSelection;
                    armTarget = armSelection;
                }
                set= false;
                clawServo.setPosition(CLAW_CLOSED);
            }
            else if (gamepad2.dpad_up){

                previousState = currentState;
                currentState = STATE.HIGH;
                armSelection = ARM_MID_TOP - armOffset;
                slideSelection= SLIDE_HIGH - liftOffset;

                if (previousState == STATE.ZERO){
                    armTarget= ARM_MID_TOP - armOffset;
                } else {
                    slideTarget= slideSelection;
                    armTarget = armSelection;
                }
                set=false;
                clawServo.setPosition(CLAW_CLOSED);

            }

            if (gamepad2.dpad_right && gamepad2.left_trigger>0.1){
                previousState = currentState;
                currentState = STATE.BACK_MID;
                slideSelection = SLIDE_MIDDLE - liftOffset;
                armSelection = ARM_BACK - armOffset;
                if (previousState == STATE.ZERO){
                    armTarget = armSelection;
                } else {
                    armTarget = armSelection;
                    slideTarget = slideSelection;
                }
                set=false;
                clawServo.setPosition(CLAW_CLOSED);
            }
            else if (gamepad2.dpad_right) {
                previousState = currentState;
                currentState = STATE.MID;
                slideSelection = SLIDE_MIDDLE - liftOffset;
                armSelection = ARM_MID_TOP - armOffset;
                if (previousState == STATE.ZERO){
                    armTarget = ARM_MID_TOP -armOffset;
                } else {
                    armTarget = ARM_MID_TOP - armOffset;
                    slideTarget = slideSelection;
                }
                set=false;
                clawServo.setPosition(CLAW_CLOSED);
            }

            if (gamepad2.dpad_down) {

                previousState = currentState;
                currentState = STATE.BOTTOM;
                armSelection = ARM_BOTTOM_JUNCTION - armOffset;
                slideSelection= SLIDE_BOTTOM - liftOffset;

                if (previousState == STATE.ZERO){
                    armTarget = ARM_BOTTOM_JUNCTION -armOffset;
                } else {
                    armTarget = ARM_BOTTOM_JUNCTION - armOffset;
                    slideTarget = SLIDE_BOTTOM;
                }
                set=false;
                clawServo.setPosition(CLAW_CLOSED);
            }

            if (gamepad2.dpad_left) {
                if (currentState!=STATE.ZERO) {
                    previousState = currentState;
                    currentState = STATE.ZERO;
                    armSelection = ARM_BOTTOM -armOffset;
                    slideSelection = SLIDE_BOTTOM - liftOffset;
                    if (previousState == STATE.BOTTOM) {
                        armTarget = armSelection;
                    } else {
                        slideTarget = slideSelection;
                    }
                }
                set=false;
                clawServo.setPosition(CLAW_CLOSED);

            }

            if (currentState == STATE.HIGH || currentState == STATE.MID || currentState == STATE.BACK_HIGH || currentState == STATE.BACK_MID){
                if (slideTarget!=slideSelection){
                    if (armMotor.getCurrentPosition()>100){
                        slideTarget = slideSelection;
                    }
                }
            } else if (currentState == STATE.ZERO && previousState!=STATE.ZERO ){

                if (linearSlideMotor.getCurrentPosition()<100 ){
                    armTarget=ARM_BOTTOM;
                }

                if (armMotor.getCurrentPosition()<200 && !set){
                    clawServo.setPosition(CLAW_OPEN);
                    tippingServo.setPosition(TIP_CENTER);
                    armTarget =0;
                    set=true;
                }
            }
            if (armMotor.getCurrentPosition()>200 && !set){
                if (currentState == STATE.BOTTOM || currentState == STATE.MID || currentState == STATE.HIGH){
                    tippingServo.setPosition(TIP_FRONT);
                    set=true;
                } else if (currentState== STATE.BACK_HIGH || currentState == STATE.BACK_MID){
                    tippingServo.setPosition(TIP_BACK);
                    set=true;
                }
            }


            if (gamepad2.right_trigger>0.3 && !trigger){
                trigger = true;
                previousState = currentState;
                currentState = STATE.CONE;
                armTarget = ARM_CONE_STACK - armOffset;
                slideSelection += SLIDE_CONE_INCREMENT;
            } else {
                trigger = false;
            }

            if (currentState == STATE.CONE){
                if (armMotor.getCurrentPosition()>100){
                    slideTarget = slideSelection;
                }
            }

            if (gamepad2.right_stick_y>0.5){
                previousState= currentState;
                currentState = STATE.MANUAL;
                if (armMotor.getCurrentPosition()>100){
                    slideTarget -= 12;
                }

            } else if (gamepad2.right_stick_y<-0/5){
                previousState= currentState;
                currentState = STATE.MANUAL;
                if (armMotor.getCurrentPosition()>100){
                    slideTarget += 12;
                }
            }

            if (gamepad2.left_stick_y>0.6){
                previousState= currentState;
                currentState = STATE.MANUAL;
                if(openClaw){
                    clawServo.setPosition(CLAW_CLOSED);
                    openClaw = false;
                }
                //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveArm = true;
               // armMotor.setPower(-0.3);

                armTarget = armTarget -8;

            }
            else if (gamepad2.left_stick_y<-0.6){
                previousState= currentState;
                currentState = STATE.MANUAL;
                if(openClaw){
                    clawServo.setPosition(CLAW_CLOSED);
                    openClaw = false;
                }
                moveArm = true;
                armTarget = armTarget+8;

            }


            moveSlideMotors();
            moveArm();
            telemetry.update();
        }
    }

    protected void moveSlideMotors (){
        liftPIDController.setTarget(slideTarget);
        linearSlideMotor.setPower(liftPIDController.calculatePower(linearSlideMotor));
        slideOtherer.setPower(liftPIDController.calculatePower(slideOtherer));
        frontSlide.setPower(liftPIDController.calculatePower(frontSlide));

//        linearSlideMotor.setTargetPosition(slideTarget);
//        frontSlide.setTargetPosition(slideTarget);
//        slideOtherer.setTargetPosition(slideTarget);
//        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideOtherer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideOtherer.setPower(1);
//        linearSlideMotor.setPower(1);
//        frontSlide.setPower(1);

    }

    protected void moveArm (){
        armPIDController.setTarget(armTarget);
        armMotor.setVelocity(armPIDController.calculateVelocity());

//        m_controller.setGoal(armTarget);
//
//        double output = m_controller.calculate(
//                armMotor.getCurrentPosition()  // the measured value
//        );
//
//        double ff = Math.cos(Math.toRadians((armTarget+400) / ticks_in_degree)) * f_arm * ticks_per_seconds;
//        telemetry.addData("output", output + ff);
//        armMotor.setVelocity(output+ ff);
//
//        //armPIDController.setTarget(armTarget);
//        double velocity = armPIDController.calculateVelocity();
//        armMotor.setPower(velocity);

       // telemetry.addData("power", velocity);
    }

    enum STATE{
        ZERO, BOTTOM, MID, HIGH, BACK_MID, BACK_HIGH, MANUAL, CONE
    }

    public boolean alignPole() {
        PowerPlayComputerVisionPipelines.PipePosition pos = CV.sleevePipeline.position;
        switch (pos) {
            case LEFT1:
            case LEFT2:
            case LEFT3:
            case LEFT4:
            case LEFT5:
            case LEFT6:
            case LEFT7:
            case LEFT8:
                leftFrontMotor.setPower(-ALIGN_SPEED);
                leftRearMotor.setPower(-ALIGN_SPEED);
                rightFrontMotor.setPower(ALIGN_SPEED);
                rightRearMotor.setPower(ALIGN_SPEED);
                break;
            case RIGHT1:
            case RIGHT2:
            case RIGHT3:
            case RIGHT4:
            case RIGHT5:
            case RIGHT6:
            case RIGHT7:
            case RIGHT8:
                leftFrontMotor.setPower(ALIGN_SPEED);
                leftRearMotor.setPower(ALIGN_SPEED);
                rightFrontMotor.setPower(-ALIGN_SPEED);
                rightRearMotor.setPower(-ALIGN_SPEED);
                break;
            case CENTER:
                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);
                return true;
        }
        return false;
    }


}