package com.kalipsorobotics.intoTheDeep;

import android.os.Process;
import android.util.Log;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.WallToBarAction;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.AutoRobotHangAction;
import com.kalipsorobotics.actions.Init;
import com.kalipsorobotics.actions.SampleEndToEndSequence;
import com.kalipsorobotics.actions.TrussSpecimenEndToEndSequence;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.drivetrain.AngleLockTeleOp;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.MoveWallTeleOp;
import com.kalipsorobotics.actions.intake.FunnelEndToEndAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.tensorflow.CameraCapture;
import com.kalipsorobotics.utilities.KGamePad;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp
public class Teleop extends LinearOpMode {

    protected boolean isRed = true;
    protected boolean takeInYellow = true;

    Action lastOuttakeAction = null;

    Action lastIntakeAction = null;

    Action lastMoveAction = null;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);

//        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        //Outtake.setInstanceNull();
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        //IntakeClaw.setInstanceNull();
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);

        //IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        //WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        //0.5 because weight gets doubled
        wheelOdometry.setWheelHeadingWeight(0.5);
        wheelOdometry.setImuHeadingWeight(0);


        DriveAction driveAction = new DriveAction(driveTrain);
        MoveWallTeleOp moveWallTeleOp = null;
        AngleLockTeleOp angleLockTeleOp = null;
        SpecimenHangReady specimenHangReady = null;
        // Target should always be 0
        MoveLSAction maintainLS = new MoveLSAction(outtake, outtake.getCurrentPosMm());
        maintainLS.setName("maintainLS");
        AutoRobotHangAction autoRobotHangAction = null;
        CameraCapture cameraCapture = new CameraCapture();
        SampleIntakeReady sampleIntakeReady = null;
        SampleIntakeAction sampleIntakeAction = null;
        IntakeTransferReady intakeTransferReady = null;
        TransferAction transferAction = null;
        BasketReadyAction basketReadyAction = null;
        OuttakeTransferReady outtakeTransferReady = null;
        SampleEndToEndSequence sampleEndToEndSequence = null;
        TrussSpecimenEndToEndSequence trussSpecimenEndToEndSequence = null;
        SampleEndToEndSequence specimenEndToEndSequence = null;
        SpecimenHang specimenHang = null;
        FunnelEndToEndAction intakeFunnelEndToEndAction = null;
        KServoAutoAction hangHook1Move = null;
        KServoAutoAction hangHook2Move = null;
        IntakeFunnelAction intakeFunnelAction = null;
        KGamePad kGamePad2 = new KGamePad(gamepad2);
        KGamePad kGamePad1 = new KGamePad(gamepad1);
        MoveLSAction moveLS = null;
        WallToBarAction wallToBarAction = null;
        OuttakePivotAction outtakePivotAction = null;

        boolean needMaintainLs = true;

        double intakeLinkagePos = IntakeClaw.INTAKE_LINKAGE_IN_POS;
        double intakeBigSweepPos = 0.5;
        double intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS;
        double intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS;
        double intakeSmallSweepPos = IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS;
        double intakeClawPos = IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN;

        double outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
        double outtakePivotPos = 0;

        // GAMEPAD 1
        boolean resetWheelOdomPressed;
        boolean hangHookUpPressed;
        boolean hangHookDownPressed;
        boolean hangPressed = false;
        boolean moveWallTeleopPressed = false;
        boolean intakeFunnelEndToEndPressed = false;
//        boolean intakeFunnelActionPressed = false;
        boolean wallToBarPressed = false;

        // GAMEPAD 2
        double outtakeLSStickValue;
        double outtakePivotStickValue;
        double intakeStickValue;
        double sweepStickValue;
        boolean outtakeOverrideOn;
        boolean specimenHangReadyPressed;
        boolean intakeClawOpenClose;
        boolean intakeClawOpenCloseWasPressed = false;
        boolean intakeOverrideOn;
        boolean intakeReadyPressed;
        boolean intakeTransferReadyPressed;
        boolean transferPressed;
        boolean intakePressed;
        boolean outtakeTransferReadyPressed;
        boolean outtakeClawOpenClose;
        boolean outtakeClawOpenCloseWasPressed = false;
        boolean basketReadyPressed;
        boolean specimenHangPressed;
        boolean specimenReadyPressed;
        boolean sampleEndToEndSequencePressed;
        boolean trussSpecimenEndToEndSequencePressed;
        boolean specimenEndToEndSequencePressed;

        //double hangPosX = SPECIMEN_HANG_POS_X+50;
        int hangPosY = 100;
        int hangIncrement = 0;
        Position savedHangPosition = null;

        Position savedWallPosition = null;

        Init init = new Init(intakeClaw, outtake);
//        outtake.init();
//        intakeClaw.init();




        ExecutorService executorService = Executors.newSingleThreadExecutor();

        waitForStart();

        executorService.submit(() -> {
            android.os.Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
            while (true) {
                wheelOdometry.updatePosition();
            }

        });

        while (opModeIsActive()) {
            while(!init.getIsDone()) {
                init.updateCheckDone();
            }

            // GAMEPAD 1 ASSIGNMENTS ==============================================
            resetWheelOdomPressed = kGamePad1.isToggleDpadUp();
            hangHookUpPressed = kGamePad1.isToggleRightBumper();
            hangHookDownPressed = kGamePad1.isRightTriggerPressed();
            hangPressed = kGamePad1.isToggleButtonB();
            moveWallTeleopPressed = kGamePad1.isToggleButtonA();
            intakeFunnelEndToEndPressed = kGamePad1.isToggleDpadLeft();
            wallToBarPressed = kGamePad1.isToggleButtonX();

            // GAMEPAD 2 ASSIGNMENTS ==============================================
            outtakeLSStickValue = gamepad2.right_stick_y;
            intakeStickValue = gamepad2.left_stick_y;
            sweepStickValue = gamepad2.left_stick_x;
            outtakeOverrideOn = kGamePad2.isRightTriggerPressed();
            specimenHangReadyPressed = kGamePad2.isToggleButtonB();
            intakeClawOpenClose = kGamePad2.isToggleLeftBumper();
            intakeOverrideOn = kGamePad2.isLeftTriggerPressed();
            intakeReadyPressed = kGamePad2.isToggleDpadLeft() && !kGamePad2.isLeftTriggerPressed();
            intakeTransferReadyPressed = kGamePad2.isToggleDpadRight();
            transferPressed = kGamePad2.isToggleDpadUp();
            intakePressed = kGamePad2.isToggleDpadDown();
            outtakeTransferReadyPressed = kGamePad2.isToggleButtonX();
            outtakeClawOpenClose = kGamePad2.isToggleRightBumper();
            outtakePivotStickValue = gamepad2.right_stick_x;
            basketReadyPressed = kGamePad2.isToggleButtonY();
            specimenHangPressed = kGamePad2.isToggleButtonA();
            specimenReadyPressed = kGamePad2.isToggleButtonB();
            sampleEndToEndSequencePressed = kGamePad2.isBackButtonPressed();
            trussSpecimenEndToEndSequencePressed = kGamePad2.isLeftTriggerPressed() && kGamePad2.isStartButtonPressed();
            specimenEndToEndSequencePressed = kGamePad2.isStartButtonPressed();

            if(kGamePad2.isLeftBumperPressed()){
                Log.d ("backButton", " Gamepad" + gamepad2.left_bumper);
                Log.d ("backButton ", " isLeftBumperPressed" + kGamePad2.isLeftBumperPressed());
                Log.d ("backButton ", " isBackButtonPressed" + kGamePad2.isBackButtonPressed());
                Log.d ("backButton ", " sampleEndToEndSequence " + trussSpecimenEndToEndSequencePressed);
            }

            //RESET POSITIONS TO CURRENT
            intakeLinkagePos = intakeClaw.getIntakeLinkageServo().getServo().getPosition();
            intakeBigSweepPos = intakeClaw.getIntakeBigSweepServo().getServo().getPosition();
            intakeBigPivotPos = intakeClaw.getIntakeBigPivotServo().getServo().getPosition();
            intakeSmallPivotPos = intakeClaw.getIntakeSmallPivotServo().getServo().getPosition();
            intakeSmallSweepPos = intakeClaw.getIntakeSmallSweepServo().getServo().getPosition();
            outtakePivotPos = outtake.getOuttakePivotServo().getServo().getPosition();

            //=========DRIVER 1==========
            if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0) {
                setLastMoveAction(null);
                driveAction.move(gamepad1);
            } else {
                if (lastMoveAction == null || lastMoveAction.getIsDone()) {
                    driveTrain.setPower(0);
                }
            }

            //RESET POS
//            if (resetWheelOdomPressed) {
//                driveTrain.resetWheelOdom();
//                wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 180);
//                Log.d("teleop_odo", "   reset odometry");
//            }

//            if (angleLockPressed) {
//                Log.d("gampad1",
//                        "gamepad1.y" + gamepad1.y + "   angle lock condition  " + (angleLockTeleOp == null || angleLockTeleOp.getIsDone()));
//
//                if (angleLockTeleOp == null || angleLockTeleOp.getIsDone()) {
//                    angleLockTeleOp = new AngleLockTeleOp(driveTrain, wheelOdometry);
//                    angleLockTeleOp.setName("angleLockTeleOp");
//
//                    setLastMoveAction(angleLockTeleOp);
//                }
//            }

            if(moveWallTeleopPressed) {
                if (moveWallTeleOp == null || moveWallTeleOp.getIsDone()){
                    moveWallTeleOp = new MoveWallTeleOp(driveTrain, wheelOdometry, null);
                    moveWallTeleOp.setName("moveWallTeleop");

                    setLastMoveAction(moveWallTeleOp);
                }

            }

            if(wallToBarPressed) {
                if (wallToBarAction == null || wallToBarAction.getIsDone()){
                    wallToBarAction = new WallToBarAction(driveTrain, wheelOdometry, null, hangIncrement);
                    wallToBarAction.setName("wallToBarHangRoundTrip");

                    hangIncrement += 75;

                    setLastMoveAction(wallToBarAction);
                }
            }


            if (!isGamePadDriveJoystickZero()) {  //cacel action b/c of Manual control override
                setLastMoveAction(null);
            }

            //HANG


            if(hangPressed) {
                if (autoRobotHangAction == null || autoRobotHangAction.getIsDone()){
//                    maintainLS = null;
                    autoRobotHangAction = new AutoRobotHangAction(outtake);
                    autoRobotHangAction.setName("autoRobotHangAction");

                    needMaintainLs = false;

                    setLastOuttakeAction(autoRobotHangAction);
                }
            }

            if(hangHookUpPressed) {
                if ((hangHook1Move == null || hangHook1Move.getIsDone()) && (hangHook2Move == null || hangHook2Move.getIsDone())) {
                    hangHook1Move = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_HANG_POS);
                    hangHook2Move = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_HANG_POS);
                }
            }

            if(hangHook1Move != null && hangHook2Move != null) {
                hangHook1Move.updateCheckDone();
                hangHook2Move.updateCheckDone();
            }

            if(hangHookDownPressed) {
                if ((hangHook1Move == null || hangHook1Move.getIsDone()) && (hangHook2Move == null || hangHook2Move.getIsDone())) {
                    hangHook1Move = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_DOWN_POS);
                    hangHook2Move = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_DOWN_POS);
                }
            }

            if(hangHook1Move != null && hangHook2Move != null) {
                hangHook1Move.updateCheckDone();
                hangHook2Move.updateCheckDone();
            }

            //INTAKE SAMPLE IN FUNNEL


            if(intakeFunnelEndToEndPressed) {
                if (intakeFunnelEndToEndAction == null || intakeFunnelEndToEndAction.getIsDone()){
                    intakeFunnelEndToEndAction = new FunnelEndToEndAction(intakeClaw, outtake);
                    intakeFunnelEndToEndAction.setName("intakeFunnelReady");
                    setLastIntakeAction(intakeFunnelEndToEndAction);
                }

            }



//            if(intakeFunnelActionPressed) {
//                if (intakeFunnelAction == null || intakeFunnelAction.getIsDone()){
//                    intakeFunnelAction = new IntakeFunnelAction(intakeClaw, outtake);
//                    intakeFunnelAction.setName("intakeFunnelAction");
//                }
//
//            }
//            if (intakeFunnelAction != null){
//                intakeFunnelAction.updateCheckDone();
//            }
//


            //===============DRIVER 2===============

            //INTAKE

            if(intakeClawOpenClose) {
                intakeClawOpenCloseWasPressed = true;
            }

            if(!intakeClawOpenClose) {
                if(intakeClawOpenCloseWasPressed) {

                    setLastIntakeAction(null);

                    if(intakeClawPos == IntakeClaw.IntakeClawConfig.INTAKE_CLAW_CLOSE) {
                        intakeClawPos = IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    } else if (intakeClawPos == IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN){
                        intakeClawPos = IntakeClaw.IntakeClawConfig.INTAKE_CLAW_CLOSE;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    }
                }

                intakeClawOpenCloseWasPressed = false;
            }



            if (-intakeStickValue != 0) {
                setLastIntakeAction(null);
                intakeClaw.getIntakeRatchetServo().setPosition(IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);

                intakeLinkagePos += 0.01 * intakeStickValue;
                if (intakeLinkagePos < 0.57) {
                    intakeLinkagePos = 0.57;
                }
                if (intakeLinkagePos > 0.95) {
                    intakeLinkagePos = 0.95;
                }
                intakeClaw.getIntakeLinkageServo().setPosition(intakeLinkagePos);
            }



            if (-sweepStickValue > 0.5 && intakeOverrideOn) {
                setLastIntakeAction(null);
                intakeBigSweepPos -= 0.005;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            } else if (-sweepStickValue < -0.5 && intakeOverrideOn) {
                setLastIntakeAction(null);
                intakeBigSweepPos += 0.005;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            }

//            if (gamepad2.y) {
//                intakeBigPivotPos += 0.005;
//            } else if (gamepad2.a) {
//                intakeBigPivotPos -= 0.005;
//            }

//            if (gamepad2.right_bumper && !gamepad2.b && !gamepad2.x) {
//                intakeSmallPivotPos += 0.005;
//            } else if (gamepad2.right_trigger > 0) {
//                intakeSmallPivotPos -= 0.005;
//            }

//            if (intakeSmallPivotPos > 1) {
//                intakeSmallPivotPos = 1;
//            } else if(intakeSmallPivotPos < 0) {
//                intakeSmallPivotPos = 0;
//            }

            if ((-sweepStickValue > 0.5) && !intakeOverrideOn) {
                setLastIntakeAction(null);
//                intakeSmallSweepPos += 0.025;
//                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
                intakeClaw.getIntakeSmallSweepServo().setPosition(IntakeClaw.INTAKE_SMALL_SWEEP_VERTICAL_POS);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            } else if ((-sweepStickValue < -0.5) && !intakeOverrideOn) {
                setLastIntakeAction(null);
//                intakeSmallSweepPos -= 0.025;
//                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
                intakeClaw.getIntakeSmallSweepServo().setPosition(IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            }

            if(intakeReadyPressed) {
                if(sampleIntakeReady != null) {
                    Log.d("teleop", "sample intake ready done: " + sampleIntakeReady.getIsDone());
                } else {
                    Log.d("teleop", "sample intake ready null");
                }

                if (sampleIntakeReady == null || sampleIntakeReady.getIsDone()){

                    //intakeClawPos default to open
                    intakeClawPos = IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN;


                    if (lastIntakeAction instanceof SampleIntakeAction) {
                        intakeClawPos = IntakeClaw.IntakeClawConfig.INTAKE_CLAW_CLOSE;
                    }

                    sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw,
                            IntakeClaw.INTAKE_SMALL_SWEEP_VERTICAL_POS, intakeClawPos);
                    Log.d("teleop", "made new sample intake  ready");
                    sampleIntakeReady.setName("sampleIntakeReady");

                    setLastIntakeAction(sampleIntakeReady);
                }

            }


            if(intakePressed) {
                if (sampleIntakeAction == null || sampleIntakeAction.getIsDone()){
                    sampleIntakeAction = new SampleIntakeAction(intakeClaw);
                    sampleIntakeAction.setName("sampleIntakeAction");
                    setLastIntakeAction(sampleIntakeAction);
                }

            }


            if (intakeTransferReadyPressed){
                if(intakeTransferReady != null) {
                    Log.d("teleop", "intake transfer ready done: " + intakeTransferReady.getIsDone());
                } else {
                    Log.d("teleop", "intake transfer ready null");
                }

                if (intakeTransferReady == null || intakeTransferReady.getIsDone()){
                    intakeTransferReady = new IntakeTransferReady(intakeClaw);
                    Log.d("teleop", "made new intake transfer ready");
                    intakeTransferReady.setName("intakeTransferReady");
                    setLastIntakeAction(intakeTransferReady);
                }

            }


            if(transferPressed) {
                if (transferAction == null || transferAction.getIsDone()){
                    transferAction = new TransferAction(intakeClaw, outtake);
                    transferAction.setName("transferAction");
                    setLastIntakeAction(transferAction);
                }

            }

            //OUTTAKE

            //outtake manual LS
            if (outtakeLSStickValue != 0) {
                if(moveLS != null) {
                    moveLS.finishWithoutSetPower();
                }
                needMaintainLs = true;
                double targetLsMM = outtake.getCurrentPosMm() + (-90.0 * outtakeLSStickValue);
                moveLS = new MoveLSAction(outtake, targetLsMM);
                moveLS.setName("moveLS");
                Log.d("ls_debug", "joystick: " + outtakeLSStickValue + " motor pos: "+ CalculateTickPer.ticksToMmLS(outtake.getLinearSlide1().getCurrentPosition()) + " setting LS target to: " + targetLsMM);

                setLastOuttakeAction(moveLS);
//                MoveOuttakeLSAction.setNeedMaintenance(true)
//                MoveOuttakeLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(15) * -outtakeLSStickValue);
            } else {
                if(moveLS != null) {
                    moveLS.finish();
                }
                moveLS = null;
            }

//            if (gamepad2.left_stick_button) {
//                //outtake slide
//                CalculateTickPer.MIN_RANGE_LS_TICKS = -2500;
//                //intake slide
//                CalculateTickPer.MIN_RANGE_INTAKE_TICKS = CalculateTickPer.degToTicksIntakeLS(-100);
//            }

            if (outtakeOverrideOn) {
                //outtake slide
                outtake.resetEncoders();

                CalculateTickPer.setMinRangeLsTicksInMm(-30); //lower min range so LS can go down faster without hitting limit

            } else {
                CalculateTickPer.resetMinRangeLsTicksToDefault(); //reset min range
            }

            if (outtakeTransferReadyPressed) {
                if (outtakeTransferReady == null || outtakeTransferReady.getIsDone()) {
                    outtakeTransferReady = new OuttakeTransferReady(outtake);
                    outtakeTransferReady.setName("outtakeTransferReady");

                    setLastOuttakeAction(outtakeTransferReady);
                }
            }

            //outtake hang ready
            if (specimenHangReadyPressed) {
                if (specimenHangReady == null || specimenHangReady.getIsDone()) {
                    specimenHangReady = new SpecimenHangReady(outtake); // extra hang = 25
                    specimenHangReady.setName("specimenHangReady");

                    setLastOuttakeAction(specimenHangReady);
                }

            }

            if(specimenHangPressed) {
                if(specimenHang == null || specimenHang.getIsDone()){
                   specimenHang = new SpecimenHang(outtake);
                   specimenHang.setName("specimenHang");

                    savedHangPosition = new Position(SharedData.getOdometryPosition().getX(),
                            SharedData.getOdometryPosition().getY(), SharedData.getOdometryPosition().getTheta());
                    Log.d("savedPositions", "hang position  " + savedHangPosition.toString());
                   //wheelOdometry.setCurrentPosition(WallToBarAction.HANG_POS, wheelOdometry.getCurrentPosition()
                    // .getY(), wheelOdometry.getCurrentPosition().getTheta());
//
//                   hangPosX = wheelOdometry.getCurrentPosition().getX();

                   setLastOuttakeAction(specimenHang);
                }
            }

            if(basketReadyPressed) {
                if (basketReadyAction == null || basketReadyAction.getIsDone()){
                    basketReadyAction = new BasketReadyAction(outtake);
                    basketReadyAction.setName("basketReadyAction");

                    setLastOuttakeAction(basketReadyAction);
                }
            }


            if(outtakeClawOpenClose) {
                outtakeClawOpenCloseWasPressed = true;
            }

            if(!outtakeClawOpenClose) {
                if(outtakeClawOpenCloseWasPressed){
                    if(outtakeClawPos == Outtake.OUTTAKE_CLAW_CLOSE) {
                        outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
                        outtake.getOuttakeClaw().setPosition(outtakeClawPos);
                    } else if (outtakeClawPos == Outtake.OUTTAKE_CLAW_OPEN){
                        outtakeClawPos = Outtake.OUTTAKE_CLAW_CLOSE;

                        savedWallPosition = new Position(SharedData.getOdometryPosition().getX(),
                                SharedData.getOdometryPosition().getY(),
                                SharedData.getOdometryPosition().getTheta());

                        Log.d("savedPositions", "wall position  " + savedWallPosition.toString());


                        outtake.getOuttakeClaw().setPosition(outtakeClawPos);
                    }

                    outtakeClawOpenCloseWasPressed = false;
                }
            }

            if (outtakePivotStickValue > 0.5) {
                if(outtakePivotAction == null || outtakePivotAction.getIsDone()) {
                    outtakePivotAction = new OuttakePivotAction(outtake, Outtake.OUTTAKE_PIVOT_WALL_READY_POS);
                    setLastOuttakeAction(outtakePivotAction);
                }
//                outtake.getOuttakePivotServo().setPosition(Outtake.OUTTAKE_PIVOT_WALL_READY_POS);
//                Log.d("sweeping",  "" + intakeSmallSweepPos);
            } else if (outtakePivotStickValue < -0.5) {
                if(outtakePivotAction == null || outtakePivotAction.getIsDone()) {
                    outtakePivotAction = new OuttakePivotAction(outtake, Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
                    setLastOuttakeAction(outtakePivotAction);
                }
//                outtake.getOuttakePivotServo().setPosition(Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
//                Log.d("sweeping",  "" + intakeSmallSweepPos);
            }

            if(sampleEndToEndSequencePressed) {
                if(sampleEndToEndSequence == null || sampleEndToEndSequence.getIsDone()){
                    sampleEndToEndSequence = new SampleEndToEndSequence(intakeClaw, outtake);
                    sampleEndToEndSequence.setName("sampleEndToEndSequence");

                    setLastIntakeAction(sampleEndToEndSequence);
                    setLastOuttakeAction(sampleEndToEndSequence);
                }
            }

            if(trussSpecimenEndToEndSequencePressed) {
                Log.d("backButton", " " + trussSpecimenEndToEndSequencePressed);
                if(trussSpecimenEndToEndSequence == null || trussSpecimenEndToEndSequence.getIsDone()){
                    trussSpecimenEndToEndSequence = new TrussSpecimenEndToEndSequence(intakeClaw, outtake);
                    trussSpecimenEndToEndSequence.setName("trussSpecimenEndToEndSequence");
                    Log.d("backButton", "Creating sequence " + trussSpecimenEndToEndSequence);

                    setLastOuttakeAction(trussSpecimenEndToEndSequence);
                    setLastIntakeAction(trussSpecimenEndToEndSequence);
                }
            }

            if (specimenEndToEndSequencePressed) {
                if (specimenEndToEndSequence == null || specimenEndToEndSequence.getIsDone()) {
                    specimenEndToEndSequence = new SampleEndToEndSequence(intakeClaw, outtake, Outtake.LS_DOWN_POS);
                    specimenEndToEndSequence.setName("specimenEndToEndSequence");


                    setLastIntakeAction(specimenEndToEndSequence);
                    setLastOuttakeAction(specimenEndToEndSequence);
                }
            }

            // Capture pictures from webcam every 500 milliseconds if holding dpad right with gamepad 1
            //DO NOT USE ALL HOLD FOR TOO LONG it will take up to much space.
//            if (gamepad1.dpad_right) {
//                cameraCapture.capture();
//            }


//            if(maintainLS != null && MoveOuttakeLSAction.getNeedMaintenance()) {
//                maintainLS.setIsDone(true);
//                maintainLS.update();
//            }

            //wheelOdometry.updatePosition();

            if(lastOuttakeAction != null) {
                lastOuttakeAction.updateCheckDone();
            }

            if(lastIntakeAction != null) {
                lastIntakeAction.updateCheckDone();
            }

            if(lastMoveAction != null) {
                lastMoveAction.updateCheckDone();
            }

            if(needMaintainLs) {
                if (lastOuttakeAction == null || lastOuttakeAction.getIsDone()) {
                    maintainLS.setIsDone(false);
                    maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
                    maintainLS.updateCheckDone();
                }
            }

            Log.d("outtakepivot", "outtake pivotPos  " + outtake.getOuttakePivotServo().getServo().getPosition());

            telemetry.addData("odometry: ", SharedData.getOdometryPosition().toString());
            telemetry.addData("big sweep pos: ", intakeBigSweepPos);
            telemetry.addData("small sweep pos: ", intakeSmallSweepPos);
            telemetry.update();
            Log.d("teleopforauto", "odometry " + SharedData.getOdometryPosition().toString());
            Log.d("teleopforauto", "big sweep " + intakeClaw.getIntakeBigSweepServo().getPosition());
            Log.d("teleopforauto", "small sweep " + intakeClaw.getIntakeSmallSweepServo().getPosition());

        }
    }

    private void setLastOuttakeAction(Action action) {
        if(lastOuttakeAction != null) {
            lastOuttakeAction.setIsDone(true);
        }
        lastOuttakeAction = action;
    }

    private void setLastIntakeAction(Action action) {
        if(lastIntakeAction != null) {
            lastIntakeAction.setIsDone(true);
        }
        lastIntakeAction = action;
    }

    private void setLastMoveAction(Action action) {
        if(lastMoveAction != null) {
            lastMoveAction.setIsDone(true);
        }
        lastMoveAction = action;
    }

    private boolean isLinearSlidesRunning(SpecimenHangReady specimenHangReady, SpecimenWallReady specimenWallReady) {
        if (specimenHangReady == null) {
            return false;
        }
        if (specimenWallReady == null) {
            return false;
        }

        if (!specimenHangReady.checkDoneCondition()) {
            return true;
        }

        if (!specimenWallReady.checkDoneCondition()) {
            return true;
        }

        return false;
    }

    private boolean isGamePadDriveJoystickZero() {
        boolean isGamePadDriveJoystickZero =
                ((gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0));
        return isGamePadDriveJoystickZero;
    }

    private boolean isGamePadOuttakeJoystickZero() {
        boolean isGamePadOuttakeJoystickZero =
                ((gamepad2.right_stick_y == 0));
        return isGamePadOuttakeJoystickZero;
    }

    private boolean isGamePadIntakeJoystickZero() {
        boolean isGamePadIntakeJoystickZero =
                ((gamepad2.right_stick_y == 0));
        return isGamePadIntakeJoystickZero;
    }

}

