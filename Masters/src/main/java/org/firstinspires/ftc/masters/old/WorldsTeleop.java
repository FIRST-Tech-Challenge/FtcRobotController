package org.firstinspires.ftc.masters.old;


import static org.firstinspires.ftc.masters.old.CSCons.servo1Down;
import static org.firstinspires.ftc.masters.old.CSCons.servo1Up;
import static org.firstinspires.ftc.masters.old.CSCons.servo2Down;
import static org.firstinspires.ftc.masters.old.CSCons.servo2Up;
import static org.firstinspires.ftc.masters.old.CSCons.transferPush;
import static org.firstinspires.ftc.masters.old.CSCons.transferUp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.old.CSCons.DriveMode;
import org.firstinspires.ftc.masters.old.CSCons.HookPosition;
import org.firstinspires.ftc.masters.old.CSCons.OuttakePosition;
import org.firstinspires.ftc.masters.old.CSCons.OuttakeState;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Disabled
@Config
@TeleOp(name = "World Teleop", group = "competition")
public class WorldsTeleop extends LinearOpMode {

    public enum TransferStatus {
        WAITING_FOR_PIXELS (100),
        MOVE_ARM (100),
        MOVE_OUTTAKE(100),
        CLOSE_FINGERS (500),
        DONE(0);

        private int waitTime;
        private TransferStatus(int waitTime){
            this.waitTime = waitTime;
        }

        public int getWaitTime() {
            return waitTime;
        }
    }

    static int target = 0;
    OuttakePosition backSlidePos = OuttakePosition.BOTTOM;

    private final double ticks_in_degrees = 384.5 / 180;

    PIDController outtakeController;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor backSlides = null;
    DcMotor otherBackSlides = null;

    DcMotor intake = null;
    Servo intakeHeight = null;

    Servo planeRaise;

    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo outtakeMovementRight;

    Servo transferServo;

    private Servo wristServo;
    private Servo outtakeServo1, outtakeServo2;
    private OuttakeWrist outtakeWristPosition;

    DigitalChannel frontBreakBeam, backBreakBeam;

    int numberOfPixelsInRamp= 0;

    double outtakeRotationTarget;

    TransferStatus currentTransferStatus;

    ElapsedTime liftFingers = new ElapsedTime();


    private enum OuttakeWrist{
        flatRight (CSCons.wristFlatRight), angleRight(CSCons.wristAngleRight), verticalDown(CSCons.wristVerticalDown),
        vertical(CSCons.wristVertical), angleLeft(CSCons.wristAngleLeft), flatLeft(CSCons.wristFlatLeft);

        double position;
        private OuttakeWrist(double position){
            this.position= position;
        }

        public double getPosition(){
            return position;
        }
    }

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.ReadyToTransfer;

    private CSCons.IntakeDirection intakeDirection = CSCons.IntakeDirection.OFF;
    private HookPosition hookPosition = HookPosition.OPEN;




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

        backSlides = hardwareMap.dcMotor.get("backSlides");
        otherBackSlides = hardwareMap.dcMotor.get("otherBackSlides");

        planeRaise = hardwareMap.servo.get("planeRaise");
        wristServo = hardwareMap.servo.get("wrist");

        outtakeServo1 = hardwareMap.servo.get("outtakeHook");
        outtakeServo2 = hardwareMap.servo.get("microHook");

        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovement = hardwareMap.servo.get("backSlideServo");
        outtakeMovementRight = hardwareMap.servo.get("backSlideServoRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeHeight = hardwareMap.servo.get("intakeServo");

        frontBreakBeam = hardwareMap.digitalChannel.get("breakBeam2");
        frontBreakBeam.setMode(DigitalChannel.Mode.INPUT);
        backBreakBeam = hardwareMap.digitalChannel.get("breakBeam1");
        backBreakBeam.setMode(DigitalChannel.Mode.INPUT);

        transferServo = hardwareMap.servo.get("transfer");



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

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);

        hookPosition = HookPosition.OPEN;
        planeRaise.setPosition(CSCons.droneFlat);

        outtakeController = new PIDController(p, i, d);
        outtakeController.setPID(p, i, d);

        outtakeServo1.setPosition(CSCons.servo1Up);
        outtakeServo2.setPosition(CSCons.servo2Up);


        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
        intakeHeight.setPosition(CSCons.intakeInit);
        transferServo.setPosition(CSCons.transferUp);
        target = backSlidePos.getTarget();

        currentTransferStatus = TransferStatus.WAITING_FOR_PIXELS;


        ElapsedTime intakeElapsedTime = null, outtakeElapsedTime = null, pickupElapsedTime = null;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean buttonPushed = false;
        boolean intakeStackButtonPushed= false;
        boolean slideNeedstoGoDown = false;
        ElapsedTime slidesElapsedTime;
        ElapsedTime buttonPushedTime=new ElapsedTime();

        waitForStart();

        runtime.reset();
        ElapsedTime elapsedTime;
        ElapsedTime colorSensorElapsedTime = null;
//        ElapsedTime stackElapsedTime = new ElapsedTime();
        ElapsedTime closeClawElapsedTime = null;

        ElapsedTime transferElapsedTime = null;

        double[] clawStack = {CSCons.clawArmGround, CSCons.clawArm2, CSCons.clawArm3, CSCons.clawArm4, CSCons.clawArm5};
        int stackPosition = 0;
        boolean wristButtonPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            backSlidesMove(target);

            if (has2Pixels() && outtakeState!=OuttakeState.ReadyToDrop && outtakeState!=OuttakeState.MoveToDrop) {
                if (currentTransferStatus== TransferStatus.WAITING_FOR_PIXELS){
                    gamepad1.rumble(5000);
                    currentTransferStatus = TransferStatus.MOVE_ARM;
                    transferElapsedTime= new ElapsedTime();
                    transferServo.setPosition(transferUp);
                }
                if (currentTransferStatus== TransferStatus.MOVE_ARM){

                    if (transferElapsedTime!=null && transferElapsedTime.milliseconds()>currentTransferStatus.getWaitTime()){
                        transferServo.setPosition(transferPush);

                        currentTransferStatus = TransferStatus.MOVE_OUTTAKE;
                        setOuttakeToPickup();
                        transferElapsedTime = new ElapsedTime();
                    }
                }
                if (currentTransferStatus == TransferStatus.MOVE_OUTTAKE && transferElapsedTime.milliseconds()>currentTransferStatus.getWaitTime()){
                    outtakeServo1.setPosition(servo1Down);
                    outtakeServo2.setPosition(servo2Down);
                    currentTransferStatus = TransferStatus.CLOSE_FINGERS;
                    transferElapsedTime = new ElapsedTime();
                    stackPosition = 5;

                    intakeHeight.setPosition(CSCons.intakeAboveTop);

                }

                if (currentTransferStatus == TransferStatus.CLOSE_FINGERS && transferElapsedTime.milliseconds()>currentTransferStatus.getWaitTime()) {
                    transferServo.setPosition(transferUp);
                    intakeDirection = CSCons.IntakeDirection.BACKWARD;
                    intake.setPower(-CSCons.speed);
                    gamepad2.rumble(5000);
                    transferElapsedTime = new ElapsedTime();
                    currentTransferStatus=TransferStatus.DONE;

                }

            } else {

                currentTransferStatus = TransferStatus.WAITING_FOR_PIXELS;
        }

            if (gamepad1.right_bumper) {
                driveMode = DriveMode.END_GAME;
            }
            if (gamepad1.left_bumper) {
                driveMode = DriveMode.NORMAL;
            }

            if (gamepad1.dpad_down) {
                target -= 15;
            }
            if (gamepad1.x) {
                backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            switch (driveMode) {
                case NORMAL:

                    drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

                    break;
                case END_GAME:
                    drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                    if (gamepad2.y) {
                        planeRaise.setPosition(CSCons.droneShooting);
                        elapsedTime = new ElapsedTime();
                        while (elapsedTime.time(TimeUnit.MILLISECONDS) < 500 && opModeIsActive()) {

                        }
                        planeRaise.setPosition((CSCons.droneFlat));
                    }

                    if (gamepad2.left_bumper) { //down
                        backSlidePos = OuttakePosition.BOTTOM;
                        target = backSlidePos.getTarget();
                    }

                    if (gamepad2.right_bumper) { //up
                        backSlidePos = OuttakePosition.HIGH;

                        target = backSlidePos.getTarget();
                        intakeHeight.setPosition(CSCons.intakeInit);
                    }

                    if (backSlides.getCurrentPosition() > 2000) {
                        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
                        outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
                        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
                    }

                    break;
                case PIXEL_SCORE:
                    drive(gamepad1.left_stick_x, 0, 0);
                    break;
            }


            //intakeSlidesMove(intakeSlideTarget);

            if (driveMode != DriveMode.END_GAME) {


                if (gamepad1.y) {
                    planeRaise.setPosition(CSCons.droneShooting);
                    elapsedTime = new ElapsedTime();
                    while (elapsedTime.time(TimeUnit.MILLISECONDS) < 500 && opModeIsActive()) {

                    }
                    planeRaise.setPosition((CSCons.droneFlat));
                }

                if (gamepad1.right_trigger >= 0.1 && !buttonPushed) {
                    //intake start/stop
                    if (intakeDirection == CSCons.IntakeDirection.OFF || intakeDirection == CSCons.IntakeDirection.BACKWARD) {
                        intakeDirection = CSCons.IntakeDirection.ON;
                        intake.setPower(CSCons.speed);
                    } else {
                        intakeDirection = CSCons.IntakeDirection.OFF;
                        intake.setPower(0);
                    }
                    buttonPushed = true;
                }
                if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger<0.1) {
                    buttonPushed = false;
                }

                if (gamepad1.left_trigger > 0.1 && !buttonPushed) {
                    if (intakeDirection == CSCons.IntakeDirection.OFF || intakeDirection == CSCons.IntakeDirection.ON) {
                        intakeDirection = CSCons.IntakeDirection.BACKWARD;
                        intake.setPower(-CSCons.speed);
                        stackPosition=5;
                        intakeHeight.setPosition(CSCons.intakeAboveTop);
                    } else {
                        intakeDirection = CSCons.IntakeDirection.OFF;
                        intake.setPower(0);
                    }
                    buttonPushed = true;
                }

                if (gamepad2.a && outtakeState!=OuttakeState.ReadyToDrop) {
                    intakeHeight.setPosition(CSCons.intakeBottom);
                    stackPosition = 0;
                }
                if (gamepad2.y && outtakeState!=OuttakeState.ReadyToDrop) {
                    stackPosition = 5;
                    intakeHeight.setPosition(CSCons.intakeAboveTop);
                }

                if (gamepad2.x && !intakeStackButtonPushed && outtakeState!=OuttakeState.ReadyToDrop) {
                    stackPosition--;
                    intakeStackButtonPushed = true;
                }

                if (gamepad2.b && !intakeStackButtonPushed && outtakeState!=OuttakeState.ReadyToDrop) {
                    stackPosition++;
                    intakeStackButtonPushed = true;
                }

                if (!gamepad2.x && !gamepad2.b && outtakeState!=OuttakeState.ReadyToDrop) {
                    intakeStackButtonPushed = false;
                }

                if (gamepad2.left_bumper && driveMode != DriveMode.END_GAME) { //down
                    target -= 10;
                }

                if (gamepad2.right_bumper && driveMode != DriveMode.END_GAME) { //up
                    target += 10;
                }

                switch (stackPosition) {
                    case 0:
                        intakeHeight.setPosition(CSCons.intakeBottom);
                        break;
                    case 1:
                        intakeHeight.setPosition(CSCons.intake2);
                        break;
                    case 2:
                        intakeHeight.setPosition(CSCons.intake3);
                        break;
                    case 3:
                        intakeHeight.setPosition(CSCons.intake4);
                        break;
                    case 4:
                        intakeHeight.setPosition(CSCons.intake5);
                        break;
                    case 5:
                        intakeHeight.setPosition(CSCons.intakeAboveTop);
                        break;
                }
            }


                    switch (outtakeState) {
                        case ReadyToTransfer:

                            if (gamepad2.left_stick_y>0.5 && Math.abs(gamepad2.left_stick_x)<0.5 ) { // if press x and hook is closed, open hook
                                outtakeMovement.setPosition(CSCons.wristOuttakePickup);
                                outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
                                outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
                                pickupElapsedTime = new ElapsedTime();

                            }
                            if (gamepad2.left_stick_y<-0.5 && Math.abs(gamepad2.left_stick_x)<0.5 ) { // closes hook with x

                                outtakeServo1.setPosition(servo1Up);
                                outtakeServo2.setPosition(servo2Up);

                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementM);
                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementM);

                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
                                outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
                            }

                            if (pickupElapsedTime!=null &&  pickupElapsedTime.milliseconds()>250){
                                outtakeServo1.setPosition(servo1Down);
                                outtakeServo2.setPosition(servo2Down);
                                pickupElapsedTime =null;
                            }


                        if (gamepad2.left_trigger > 0.5) {
                            backSlidePos = OuttakePosition.LOW;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;
                            transferServo.setPosition(transferUp);

                        }
                        if (gamepad2.right_trigger > 0.5) {
                            backSlidePos = OuttakePosition.MID;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;
                            transferServo.setPosition(transferUp);
                        }

                            if (gamepad2.dpad_left ) {
//                                if (outtakeWristPosition == OuttakeWrist.angleRight) {
//                                    outtakeWristPosition = OuttakeWrist.flatRight;
//                                } else {
                                    outtakeWristPosition = OuttakeWrist.angleRight;
//                                }
//                                wristButtonPressed = true;
//                                buttonPushedTime= new ElapsedTime();
                            } else if (gamepad2.dpad_up ) {
                                outtakeWristPosition = OuttakeWrist.vertical;
//                                wristButtonPressed = true;
//                                buttonPushedTime= new ElapsedTime();
                            } else if (gamepad2.dpad_right ) {
//                                if (outtakeWristPosition == OuttakeWrist.angleLeft) {
//                                    outtakeWristPosition = OuttakeWrist.flatLeft;
//                                } else {
                                    outtakeWristPosition = OuttakeWrist.angleLeft;
                                //}
//                                wristButtonPressed = true;
//                                buttonPushedTime = new ElapsedTime();
                            } else if (gamepad2.dpad_down ) {
                                outtakeWristPosition = OuttakeWrist.verticalDown;
//                                wristButtonPressed = true;
//                                buttonPushedTime = new ElapsedTime();
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x<-0.1){
                                outtakeWristPosition = OuttakeWrist.flatRight;
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x>0.1){
                                outtakeWristPosition= OuttakeWrist.flatLeft;
                            }


                            break;
                        case MoveToTransfer:
                            if (backSlides.getCurrentPosition() < 50) {
                              liftOuttakeFingers();
                                outtakeState = OuttakeState.ReadyToTransfer;
                                target = 0;
                            }


                            //if touch sensor => ready to transfer
                            break;
                        case ReadyToDrop:

                            if (gamepad2.dpad_left ) {
                                outtakeWristPosition = OuttakeWrist.angleRight;

                            } else if (gamepad2.dpad_up ) {
                                outtakeWristPosition = OuttakeWrist.vertical;
                            } else if (gamepad2.dpad_right ) {

                                outtakeWristPosition = OuttakeWrist.angleLeft;

                            } else if (gamepad2.dpad_down ) {
                                outtakeWristPosition = OuttakeWrist.verticalDown;

                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x<-0.1){
                                outtakeWristPosition = OuttakeWrist.flatRight;
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x>0.1){
                                outtakeWristPosition= OuttakeWrist.flatLeft;
                            }

                           setWristServoPosition();

                            if(gamepad2.y){
                                if (outtakeWristPosition== OuttakeWrist.vertical){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition==OuttakeWrist.angleLeft){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition==OuttakeWrist.angleRight){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.verticalDown){
                                    outtakeServo1.setPosition(servo1Up);
                                }
                            }

                            if (gamepad2.a){
                                if (outtakeWristPosition== OuttakeWrist.vertical){
                                    outtakeServo1.setPosition(servo1Up);
                                }
                                if (outtakeWristPosition== OuttakeWrist.verticalDown){
                                    outtakeServo2.setPosition(servo2Up);
                                }
                            }

                            if (gamepad2.b){
                                if (outtakeWristPosition== OuttakeWrist.flatLeft){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.flatRight){
                                    outtakeServo1.setPosition(servo1Up);
                                } else if (outtakeWristPosition==OuttakeWrist.angleRight) {
                                    outtakeServo1.setPosition(servo1Up);
                                }
                            }

                            if (gamepad2.x){
                                if (outtakeWristPosition== OuttakeWrist.flatRight){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.flatLeft){
                                    outtakeServo1.setPosition(servo1Up);
                                }else if (outtakeWristPosition==OuttakeWrist.angleLeft){
                                    outtakeServo1.setPosition(servo1Up);
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

                                liftOuttakeFingers();
                                setOuttakeToTransfer();

                                //setup slides
                                if (backSlides.getCurrentPosition() > OuttakePosition.LOW.getTarget() + 400) {
                                    backSlidePos = OuttakePosition.BOTTOM;
                                    target = backSlidePos.getTarget();
                                    outtakeState = OuttakeState.MoveToTransfer;
                                } else if (!slideNeedstoGoDown) {
                                    outtakeElapsedTime = new ElapsedTime();
                                    slideNeedstoGoDown = true;
                                }
                            }

                            if (gamepad2.right_stick_y>0.2){
                                liftOuttakeFingers();
                                liftFingers= new ElapsedTime();
                            }
                            if (gamepad2.right_stick_y<-0.2  && liftFingers.milliseconds()>200){
                                outtakeServo1.setPosition(servo1Down);
                                outtakeServo2.setPosition(servo2Down);
                            }



                            if (slideNeedstoGoDown && outtakeElapsedTime != null && outtakeElapsedTime.milliseconds() > 500) {
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
                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
                                outtakeRotationTarget = CSCons.wristOuttakeAngleBackdrop;
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


//            if (gamepad1.a ) {
//                clawPosition = ClawPosition.TRANSFER;
//                clawServo.setPosition(CSCons.clawTransfer);
//            }

//            telemetry.addData("left y", gamepad1.left_stick_y);
//            telemetry.addData("left x", gamepad1.left_stick_x);
//            telemetry.addData("right x", gamepad1.right_stick_x);
//            telemetry.addData("Arm", clawArm.getPosition());
//            telemetry.addData("backSlides", backSlides.getCurrentPosition()) ;
//            telemetry.addData("intakeSides", intakeSlides.getCurrentPosition());// 1725, 2400,
//            telemetry.addData("time", runtime.time());
//            telemetry.addData("Status", colorSensor.status());
//            telemetry.addData("Outtake state", outtakeState.name());
//            telemetry.addData("Intake state", intakeState.name());
//            telemetry.addData("back left", leftRearMotor.getCurrentPosition());
//            telemetry.addData("back right", rightRearMotor.getCurrentPosition());
//            telemetry.addData("front left", leftFrontMotor.getCurrentPosition());
//            telemetry.addData("front right", rightFrontMotor.getCurrentPosition());
//            telemetry.addData("TOUCH:", touchBucket.isPressed());
//            telemetry.update();

            telemetry.addData("Wrist Position: ", outtakeWristPosition.name());


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

        double leftFrontPower = y + x*CSCons.frontMultiplier + rx;
        double leftRearPower = y - (x*CSCons.backMultiplier) + rx;
        double rightFrontPower = y - x*CSCons.frontMultiplier - rx;
        double rightRearPower = y + (x*CSCons.backMultiplier) - rx;

        //if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(rightRearPower));

            leftFrontPower /= denominator;
            leftRearPower /= denominator;
            rightFrontPower /= denominator;
            rightRearPower /= denominator;
        //}

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


    protected ElapsedTime closeHook(){
        ElapsedTime time = new ElapsedTime();
        hookPosition = HookPosition.CLOSED;
//        outtakeServo1.setPosition(servo1Down);
//        outtakeServo2.setPosition(servo2Down);
        return time;
    }

    protected void liftOuttakeFingers(){
        outtakeServo1.setPosition(servo1Up);
        outtakeServo2.setPosition(servo2Up);
    }

    protected void setOuttakeToTransfer(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    protected void setOuttakeToPickup(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
        outtakeMovement.setPosition(CSCons.wristOuttakePickup);
        outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    protected void setWristServoPosition(){
        if (outtakeWristPosition == OuttakeWrist.angleLeft) {
            wristServo.setPosition(CSCons.wristAngleLeft);
        }
        if (outtakeWristPosition == OuttakeWrist.angleRight) {
            wristServo.setPosition(CSCons.wristAngleRight);
        }
        if (outtakeWristPosition == OuttakeWrist.vertical) {
            wristServo.setPosition(CSCons.wristVertical);
        }
        if (outtakeWristPosition == OuttakeWrist.flatLeft) {
            wristServo.setPosition(CSCons.wristFlatLeft);
        }
        if (outtakeWristPosition == OuttakeWrist.flatRight) {
            wristServo.setPosition(CSCons.wristFlatRight);
        }
        if (outtakeWristPosition == OuttakeWrist.verticalDown) {
            wristServo.setPosition(CSCons.wristVerticalDown);
        }
    }

    protected boolean has2Pixels(){
        return !frontBreakBeam.getState() && !backBreakBeam.getState();
    }

}
