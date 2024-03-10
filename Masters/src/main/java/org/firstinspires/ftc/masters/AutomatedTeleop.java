package org.firstinspires.ftc.masters;


import static org.firstinspires.ftc.masters.CSCons.clawOpen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons.*;


import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Center Stage Automated TeleOp", group = "competition")
public class AutomatedTeleop extends LinearOpMode {

    static int target = 0;
    OuttakePosition backSlidePos = OuttakePosition.BOTTOM;

    private final double ticks_in_degrees = 384.5 / 180;

    PIDController outtakeController;
    PIDController intakeController;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

    public static double ip = 0.01, ii = 0, iid = 0.00;
    public static double iif = 0.05;

//    double y = 0;
//    double x = 0;
//    double rx = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor intakeSlides = null;
    DcMotor backSlides = null;
    DcMotor otherBackSlides = null;
    DcMotor hangingMotor = null;

    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo microHook;

    TouchSensor touchBucket;

    boolean clawClosed = true;
    boolean hookEngaged = false;
    boolean hookPressed = false;
    boolean presetPushed = false;
    double claw2transfer_time = -1;
    double claw2transfer_delay = .5;
    double x_last_pressed = -1;


    int backSlidesTargetPos = 0;
    int presetBackSlidesTargetPos = 0;
    double bucketMovedBy = 0;
    boolean outtakeGoingToTransfer;
    double outtakeRotationTarget;
    int intakeSlideTarget =0;

    private double claw_last_opened;

    private enum Retract {
        back,
        flip_bar,
        cartridge
    }

    private boolean isRetracting = false;

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.ReadyToTransfer;
    private IntakeState intakeState = IntakeState.Transition;

    private Retract retract = Retract.back;

    private ClawPosition clawPosition = ClawPosition.OPEN;
    private HookPosition hookPosition = HookPosition.OPEN;
    RevColorSensorV3 colorSensor;
    double angleRotationAdjustment = 0;

    @Override
    public void runOpMode() {
        /*
            Controls
            --------
            Gamepad2

            Gamepad1

            --------
    Drive with sticks intake is the front
    Four bar position 1-5 auto stack pixels
    Slides position 1-11 placement on backboard
    Normal mode - pixel grabbing mode - entered with left stick down
    TODO: D-pad controls extendo
    TODO: D-pad up - extendo fully extended with four bar in position 4
    TODO: D-pad right - extendo out 2/3? with four bar in position 1
    TODO: D-pad down - extendo out 1/3? with four bar in position 1
    TODO: D-pad left - extendo fully in and four bar in position 1
    Buttons
    TODO: A - claw opens and closes
    TODO: B - transfers and moves to slide pos 1
    TODO: X - auto aligns and switches to pixel scoring mode
    TODO: Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
    TODO: LB - four bar down (presets)
    TODO: RB - four bar up (presets)
    TODO: LT - slides down (presets)
    TODO: RT - slides up (presets)

    Pixel scoring mode - entered with x while in normal
    Only moves right and left no forward/ backwards
    TODO: LT - slides down (presets)
    TODO: RT - slides up (presets)
    TODO: Y - press once drop one pixel, hold for drop both pixels

    Endgame mode - entered with right stick down
    Normal driving
    TODO: X - Auto aligns on April tag for shooter, raise shooter, shoot
    TODO: LB - Hang down
    TODO: RB - Hang up
        */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");
        backSlides = hardwareMap.dcMotor.get("backSlides");
        otherBackSlides=hardwareMap.dcMotor.get("otherBackSlides");

        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        //cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        microHook = hardwareMap.servo.get("microHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovement = hardwareMap.servo.get("backSlideServo");
        touchBucket = hardwareMap.touchSensor.get("touchBucket");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");


        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        otherBackSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //backSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otherBackSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        intakeSlides.setTargetPosition(0);
//        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeSlides.setPower(.5);

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawArm.setPosition(CSCons.clawArmTransition);
        clawAngle.setPosition(CSCons.clawAngleTransition);
        clawServo.setPosition(CSCons.clawOpen);

        outtakeMovement.setPosition(CSCons.outtakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
        outtakeHook.setPosition(CSCons.openHook);
        microHook.setPosition(CSCons.openMicroHook);
        hookPosition = HookPosition.OPEN;
        planeRaise.setPosition(CSCons.droneFlat);

        outtakeController = new PIDController(p, i, d);
        outtakeController.setPID(p, i, d);
        intakeController = new PIDController(ip, ii, iid);

        target = backSlidePos.getTarget();
        angleRotationAdjustment = 0;

        ElapsedTime intakeElapsedTime = null, outtakeElapsedTime = null;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean buttonPushed= false;
        boolean slideNeedstoGoDown = false;
        ElapsedTime slidesElapsedTime;
        intakeSlides.setTargetPosition(0);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlides.setPower(0.5);

        waitForStart();

        runtime.reset();
        ElapsedTime elapsedTime;
        ElapsedTime colorSensorElapsedTime = null;
        ElapsedTime closeClawElapsedTime=null;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("intake slide position",  + intakeSlides.getCurrentPosition());
            telemetry.addData("intake slide target",  + intakeSlideTarget);

            if (gamepad1.right_bumper){
                driveMode= DriveMode.END_GAME;
            }
            if (gamepad1.left_bumper){
                driveMode= DriveMode.NORMAL;
            }
            switch (driveMode) {
                case NORMAL:
                    if (colorSensorElapsedTime != null) {
                        if (colorSensorElapsedTime.milliseconds() < 450) {
                            drive(0,0,0);
                        } else {
                            drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                        }
                    } else {
                        drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                    }
                    break;
                case END_GAME:
                    drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                    if (gamepad2.y){
                        planeRaise.setPosition(CSCons.droneShooting);
                         elapsedTime= new ElapsedTime();
                        while (elapsedTime.time(TimeUnit.MILLISECONDS)<500 && opModeIsActive()){

                        }
                        planeRaise.setPosition((CSCons.droneFlat));
                    }

                    if (gamepad2.left_bumper){ //down
                        backSlidePos = OuttakePosition.BOTTOM;
                        target = backSlidePos.getTarget();
                    }

                    if (gamepad2.right_bumper){ //up
                        backSlidePos = OuttakePosition.MID;
                        target = backSlidePos.getTarget();
                    }

                    if (backSlides.getCurrentPosition()>2000){
                        outtakeRotation.setPosition(CSCons.outtakeAngleFolder);
                        outtakeMovement.setPosition(CSCons.outtakeMovementBackDrop);
                    }

                    break;
                case PIXEL_SCORE:
                    drive(gamepad1.left_stick_x, 0, 0);
                    break;
            }

            backSlidesMove(target);
        //    intakeSlidesMove(intakeSlideTarget);

            if (driveMode!=DriveMode.END_GAME) {

                switch (intakeState) {
                    case Intake:
                        if (gamepad2.a && !buttonPushed) {
                            buttonPushed = true;
                            if (clawPosition == ClawPosition.OPEN || clawPosition == ClawPosition.TRANSFER) {
                                clawPosition = ClawPosition.CLOSED;
                                clawServo.setPosition(CSCons.clawClosed);
                                closeClawElapsedTime = new ElapsedTime();

                            } else if (clawPosition == ClawPosition.CLOSED) {
                                clawPosition = ClawPosition.OPEN;
                                clawServo.setPosition(clawOpen);
                                claw_last_opened = runtime.time(TimeUnit.MILLISECONDS);
                                closeClawElapsedTime=null;
                            }
                        }
                        if (!gamepad2.a){
                            buttonPushed = false;
                        }

                        if (clawPosition == ClawPosition.OPEN && detectPixel()) {
                            clawPosition = ClawPosition.CLOSED;
                            clawServo.setPosition(CSCons.clawClosed);

                            intakeSlideTarget = intakeSlides.getCurrentPosition();

                            intakeSlides.setTargetPosition(intakeSlides.getCurrentPosition());
                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            intakeSlides.setPower(0.5);
                            colorSensorElapsedTime = new ElapsedTime();
                        }

                        if (gamepad2.dpad_up) {
                            if (intakeSlides.getCurrentPosition() > 50) {
                                intakeSlideTarget=0;

                                intakeSlides.setTargetPosition(0);
                                intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                intakeSlides.setPower(1);
                                clawAngle.setPosition(CSCons.clawAngleTransition);
                                clawArm.setPosition(CSCons.clawArmTransition);
                                intakeState = IntakeState.MoveToTransfer;
                            } else {
                                intakeState = IntakeState.Transfer;
                                clawAngle.setPosition(CSCons.clawAngleTransfer);
                                clawArm.setPosition(CSCons.clawArmTransfer);
                            }

                        }

                        if (gamepad2.dpad_left) {
                            intakeState = IntakeState.Transition;
                            clawAngle.setPosition(CSCons.clawAngleTransition);
                            clawArm.setPosition(CSCons.clawArmTransition);
                        }

                        if (gamepad1.right_trigger>0.1) {
                            intakeSlideTarget = 1700;
//                            intakeSlides.setTargetPosition(1700);
//                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            intakeSlides.setPower(1);
                        }

                        if (gamepad1.left_trigger>0.1) {
                            intakeSlideTarget =0;
//                            intakeSlides.setTargetPosition(0);
//                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            intakeSlides.setPower(1);
                        }

                        if (clawPosition == ClawPosition.CLOSED && ((colorSensorElapsedTime!=null && colorSensorElapsedTime.milliseconds()>300)
                                ||(closeClawElapsedTime!=null && closeClawElapsedTime.milliseconds()>300))){
                            if (intakeSlides.getCurrentPosition() > 50) {
                                intakeSlideTarget=0;

                                intakeSlides.setTargetPosition(0);
                                intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                intakeSlides.setPower(1);
                                clawAngle.setPosition(CSCons.clawAngleTransition);
                                clawArm.setPosition(CSCons.clawArmTransition);
                                intakeState = IntakeState.MoveToTransfer;
                                colorSensorElapsedTime=null;
                            } else {
                                intakeState = IntakeState.Transfer;
                                clawAngle.setPosition(CSCons.clawAngleTransfer);
                                clawArm.setPosition(CSCons.clawArmTransfer);
                            }
                        }


                        break;
                    case MoveToTransfer:
                        closeClawElapsedTime=null;
                        colorSensorElapsedTime=null;
                        if (intakeSlides.getCurrentPosition() < 80) {
                            intakeState = IntakeState.Transfer;
                            clawAngle.setPosition(CSCons.clawAngleTransfer);
                            clawArm.setPosition(CSCons.clawArmTransfer);
                        }
                        if (gamepad1.right_trigger>0.1) {
                            intakeSlideTarget = 1700;
                            intakeSlides.setTargetPosition(1700);
                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            intakeSlides.setPower(1);
                            intakeState = IntakeState.Intake;
                        }
                        break;
                    case MoveToIntake:
                        if (intakeElapsedTime != null && intakeElapsedTime.time(TimeUnit.MILLISECONDS) > CSCons.transferToBottomIntake) {
                            intakeState = IntakeState.Intake;
                            clawPosition = ClawPosition.OPEN;
                            clawServo.setPosition(clawOpen);
                        }


                        break;
                    case Transfer:
                        //is there a reason we would ever want to close the claw here?
                        if (gamepad1.a) {
                            clawPosition = ClawPosition.TRANSFER;
                            clawServo.setPosition(CSCons.clawTransfer);
                        }


                        if (gamepad2.dpad_down) {
                            intakeState = IntakeState.MoveToIntake;
                            intakeElapsedTime = new ElapsedTime();
                            clawAngle.setPosition(CSCons.clawAngleGroundToThree);
                            clawArm.setPosition(CSCons.clawArmGround);
                        }

                        if (gamepad2.dpad_left) {
                            intakeState = IntakeState.Transition;
                            clawAngle.setPosition(CSCons.clawAngleTransition);
                            clawArm.setPosition(CSCons.clawArmTransition);
                        }

                        if (gamepad1.dpad_down) {
                            intakeSlideTarget=0;
                            intakeSlides.setTargetPosition(0);
                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            intakeSlides.setPower(1);
                        }


                        break;
                    case Transition:

                        if (gamepad2.dpad_down) {
                            intakeState = IntakeState.MoveToIntake;
                            intakeElapsedTime = new ElapsedTime();
                            clawAngle.setPosition(CSCons.clawAngleGroundToThree);
                            clawArm.setPosition(CSCons.clawArmGround);
                        }


                        if (gamepad2.dpad_up) {
                            intakeState = IntakeState.Transfer;
                            clawAngle.setPosition(CSCons.clawAngleTransfer);
                            clawArm.setPosition(CSCons.clawArmTransfer + CSCons.skewampus);
                        }

                        if (gamepad1.dpad_down) {
                            intakeSlideTarget =0;
                            intakeSlides.setTargetPosition(0);
                            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            intakeSlides.setPower(1);
                        }

                        break;
                }
                if (gamepad2.dpad_right) {
                    outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
                    outtakeMovement.setPosition(CSCons.outtakeMovementTransfer);
                }
                switch (outtakeState) {
                    case ReadyToTransfer:
//                        if (!touchBucket.isPressed()) {
//                            bucketMovedBy = bucketMovedBy + .01;
//                            outtakeRotation.setPosition(bucketMovedBy + CSCons.outtakeAngleTransfer);
//                        }
                        if (gamepad2.x && hookPosition == HookPosition.CLOSED) { // if press x and hook is closed, open hook
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = new ElapsedTime();
                                hookPosition = HookPosition.OPEN;
                                outtakeHook.setPosition(CSCons.openHook);
                                microHook.setPosition(CSCons.openMicroHook);
                            } else {
                                outtakeHook.setPosition(CSCons.closeHook);
                                microHook.setPosition(CSCons.closeMicroHook);
                            }
                        }
                        if (gamepad2.x && hookPosition == HookPosition.OPEN) { // closes hook with x
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = closeHook();
                            } else {
                                outtakeHook.setPosition(CSCons.openHook);
                                microHook.setPosition(CSCons.openMicroHook);
                            }
                        }

                        if (gamepad2.left_trigger > 0.5) {
                            backSlidePos = OuttakePosition.LOW;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;

                        }
                        if (gamepad2.right_trigger > 0.5) {
                            backSlidePos = OuttakePosition.MID;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;
                        }


                        break;
                    case MoveToTransfer:
                        if (backSlides.getCurrentPosition() < 50) {
                            outtakeHook.setPosition(CSCons.openHook);
                            microHook.setPosition(CSCons.openMicroHook);
                            outtakeState = OuttakeState.ReadyToTransfer;
                            target = 0;
                        }


                        //if touch sensor => ready to transfer
                        break;
                    case ReadyToDrop:
                        if (gamepad2.x && (hookPosition == HookPosition.CLOSED || hookPosition == HookPosition.HALF)) {
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = new ElapsedTime();
                                hookPosition = HookPosition.OPEN;
                                outtakeHook.setPosition(CSCons.openHook);
                                microHook.setPosition(CSCons.openMicroHook);
                            } else {
                                outtakeHook.setPosition(CSCons.closeHook);
                                microHook.setPosition(CSCons.closeMicroHook);

                            }

                        }
                        if (gamepad2.y && hookPosition == HookPosition.CLOSED) {
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = new ElapsedTime();
                                hookPosition = HookPosition.HALF;
                                outtakeHook.setPosition(CSCons.openHook);

                            }
                        }

                        if (gamepad2.y && hookPosition == HookPosition.HALF) {
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = new ElapsedTime();
                                hookPosition = HookPosition.OPEN;
                                outtakeHook.setPosition(CSCons.openHook);
                                microHook.setPosition(CSCons.openMicroHook);

                            }
                        }


                        if (gamepad2.x && hookPosition == HookPosition.OPEN) {
                            if (outtakeElapsedTime == null || outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > 300) {
                                outtakeElapsedTime = closeHook();
                            } else {
                                outtakeHook.setPosition(CSCons.openHook);
                            }
                        }

                        if (gamepad2.left_trigger > 0.5) {
                            backSlidePos = OuttakePosition.LOW;
                        }
                        if (gamepad2.right_trigger > 0.5) {
                            backSlidePos = OuttakePosition.MID;
                            target = backSlidePos.getTarget();
                        }

                        if (gamepad2.left_stick_y > 0.2) {

                            outtakeHook.setPosition(CSCons.openHook);
                            outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
                            outtakeMovement.setPosition(CSCons.outtakeMovementTransfer);
                            if (backSlides.getCurrentPosition()> OuttakePosition.LOW.getTarget()+100) {
                                backSlidePos = OuttakePosition.BOTTOM;
                                target = backSlidePos.getTarget();
                                outtakeState = OuttakeState.MoveToTransfer;
                            } else if (!slideNeedstoGoDown){
                                outtakeElapsedTime = new ElapsedTime();
                                slideNeedstoGoDown = true;
                            }
                        }

                        if (gamepad2.left_bumper && driveMode != DriveMode.END_GAME) { //down
                            target -= 15;
                        }

                        if (gamepad2.right_bumper && driveMode != DriveMode.END_GAME) { //up
                            target += 15;
                        }

                        if (slideNeedstoGoDown && outtakeElapsedTime!=null && outtakeElapsedTime.milliseconds()>500){
                            backSlidePos = OuttakePosition.BOTTOM;
                            target = backSlidePos.getTarget();
                            outtakeState = OuttakeState.MoveToTransfer;
                            slideNeedstoGoDown = false;
                        }


                        //what button to mode back to transfer?
                        // what order to move (slide down or flip first?

                        break;
                    case MoveToDrop:
                        if (backSlides.getCurrentPosition() > 100) {
                            outtakeMovement.setPosition(CSCons.outtakeMovementBackDrop);
                            outtakeRotationTarget = CSCons.outtakeAngleFolder + angleRotationAdjustment;
                            outtakeRotation.setPosition(outtakeRotationTarget);
                        }
                        if (backSlides.getCurrentPosition() > backSlidePos.getTarget() - 100) {
                            outtakeState = OuttakeState.ReadyToDrop;

                        }
                        break;
                    case Align:
                        break;
                    case BackUp:
                        break;
                    case ClosingHook:
                        if (outtakeElapsedTime != null && outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > CSCons.transferToScoreOuttake) {
                            outtakeState = OuttakeState.MoveToDrop;
                            target = backSlidePos.getTarget();
                        }
                        break;
                }
            }

//            if (gamepad1.a ) {
//                clawPosition = ClawPosition.TRANSFER;
//                clawServo.setPosition(CSCons.clawTransfer);
//            }

            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);
            telemetry.addData("Arm", clawArm.getPosition());
            telemetry.addData("backSlides", backSlides.getCurrentPosition()) ;
            telemetry.addData("intakeSides", intakeSlides.getCurrentPosition());// 1725, 2400,
            telemetry.addData("time", runtime.time());
            telemetry.addData("Status", colorSensor.status());
            telemetry.addData("Outtake state", outtakeState.name());
            telemetry.addData("Intake state", intakeState.name());
            telemetry.addData("back left", leftRearMotor.getCurrentPosition());
            telemetry.addData("back right", rightRearMotor.getCurrentPosition());
            telemetry.addData("front left", leftFrontMotor.getCurrentPosition());
            telemetry.addData("front right", rightFrontMotor.getCurrentPosition());
            telemetry.addData("TOUCH:", touchBucket.isPressed());
            telemetry.update();

            telemetry.update();
        }
    }

    protected void drive(double x, double y, double rx) {

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - (x*.69) + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + (x*.69) - rx;

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

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    protected void backSlidesMove(int target) {

        int slidePos = backSlides.getCurrentPosition();
        double pid = outtakeController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double liftPower = pid + ff;

        backSlides.setPower(liftPower);
        otherBackSlides.setPower(liftPower);
    }

    public void intakeSlidesMove(int target) {


        int slidePos = intakeSlides.getCurrentPosition();
        double pid = intakeController.calculate(slidePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * iif;

        double liftPower = pid + ff;

        intakeSlides.setPower(liftPower);
        telemetry.addData("intake power",  + liftPower);


    }

    protected ElapsedTime closeHook(){
        ElapsedTime time = new ElapsedTime();
        hookPosition = HookPosition.CLOSED;
        outtakeHook.setPosition(CSCons.closeHook);
        microHook.setPosition(CSCons.closeMicroHook);
        return time;
    }

    protected boolean detectPixel(){
        if (colorSensor.getRawLightDetected() > CSCons.pixelDetectThreshold && runtime.time(TimeUnit.MILLISECONDS) > claw_last_opened + 1000){
            return true;
        } else {
            return false;
        }
    }

}
