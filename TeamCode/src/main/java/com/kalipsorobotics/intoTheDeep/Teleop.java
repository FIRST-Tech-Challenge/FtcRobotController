package com.kalipsorobotics.intoTheDeep;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.AutoRobotHangAction;
import com.kalipsorobotics.actions.Init;
import com.kalipsorobotics.actions.SampleEndToEndSequence;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.WallToBarHangRoundTripTeleOp;
import com.kalipsorobotics.actions.drivetrain.AngleLockTeleOp;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.MoveWallTeleOp;
import com.kalipsorobotics.actions.intake.FunnelEndToEndAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.tensorflow.CameraCapture;
import com.kalipsorobotics.utilities.KGamePad;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {

    protected boolean isRed = true;
    protected boolean takeInYellow = true;

    Action lastLSAction = null;

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
        SampleEndToEndSequence specimenEndToEndSequence = null;
        SpecimenHang specimenHang = null;
        FunnelEndToEndAction intakeFunnelEndToEndAction = null;
        IntakeFunnelAction intakeFunnelAction = null;
        KGamePad kGamePad2 = new KGamePad(gamepad2);
        KGamePad kGamePad1 = new KGamePad(gamepad1);
        MoveLSAction moveLS = null;
        WallToBarHangRoundTripTeleOp wallToBarHangRoundTripTeleOp = null;

        boolean needMaintainLs = true;

        double intakeLinkagePos = IntakeClaw.INTAKE_LINKAGE_IN_POS;
        double intakeBigSweepPos = 0.5;
        double intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS;
        double intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS;
        double intakeSmallSweepPos = IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS;
        double intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;

        double outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
        double outtakePivotPos = 0;

        // GAMEPAD 1
        boolean resetWheelOdomPressed;
        boolean angleLockPressed;
        boolean hangPressed = false;
        boolean moveWallTeleopPressed = false;
        boolean intakeFunnelEndToEndPressed = false;
//        boolean intakeFunnelActionPressed = false;
        boolean specimenRoundTripPressed = false;

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
        boolean specimenEndToEndSequencePressed;

        int hangPos = 100;
        int hangIncrement = 60;

//        Init init = new Init(intakeClaw, outtake);
//        while(opModeInInit()) {
//            init.updateCheckDone();
//        }
        outtake.init();
        intakeClaw.init();

        waitForStart();

        while (opModeIsActive()) {
            // GAMEPAD 1 ASSIGNMENTS ==============================================
            resetWheelOdomPressed = kGamePad1.isToggleDpadUp();
            angleLockPressed = kGamePad1.isToggleButtonY();
            hangPressed = kGamePad1.isToggleButtonB();
            moveWallTeleopPressed = kGamePad1.isToggleButtonA();
            intakeFunnelEndToEndPressed = kGamePad1.isToggleDpadLeft();
            specimenRoundTripPressed = kGamePad1.isToggleButtonX();

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
            specimenEndToEndSequencePressed = kGamePad2.isToggleDpadLeft() && kGamePad2.isLeftTriggerPressed();

            if(kGamePad2.isLeftBumperPressed()){
                Log.d ("backButton", " Gamepad" + gamepad2.left_bumper);
                Log.d ("backButton ", " isLeftBumperPressed" + kGamePad2.isLeftBumperPressed());
                Log.d ("backButton ", " isBackButtonPressed" + kGamePad2.isBackButtonPressed());
                Log.d ("backButton ", " sampleEndToEndSequence " + specimenEndToEndSequencePressed);
            }

            //RESET POSITIONS TO CURRENT
            intakeLinkagePos = intakeClaw.getIntakeLinkageServo().getServo().getPosition();
            intakeBigSweepPos = intakeClaw.getIntakeBigSweepServo().getServo().getPosition();
            intakeBigPivotPos = intakeClaw.getIntakeBigPivotServo().getServo().getPosition();
            intakeSmallPivotPos = intakeClaw.getIntakeSmallPivotServo().getServo().getPosition();
            intakeSmallSweepPos = intakeClaw.getIntakeSmallSweepServo().getServo().getPosition();
            outtakePivotPos = outtake.getOuttakePivotServo().getServo().getPosition();

            //=========DRIVER 1==========
            if(wallToBarHangRoundTripTeleOp == null || wallToBarHangRoundTripTeleOp.getIsDone()) {
                driveAction.move(gamepad1);
            }

            //RESET POS
            if (resetWheelOdomPressed) {
                driveTrain.resetWheelOdom();
                wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 180);
                Log.d("teleop_odo", "   reset odometry");
            }

            if (angleLockPressed) {
                Log.d("gampad1",
                        "gamepad1.y" + gamepad1.y + "   angle lock condition  " + (angleLockTeleOp == null || angleLockTeleOp.getIsDone()));

                if (angleLockTeleOp == null || angleLockTeleOp.getIsDone()) {
                    angleLockTeleOp = new AngleLockTeleOp(driveTrain, wheelOdometry);
                    angleLockTeleOp.setName("angleLockTeleOp");
                }
            }

            if(moveWallTeleopPressed) {
                if (moveWallTeleOp == null || moveWallTeleOp.getIsDone()){
                    moveWallTeleOp = new MoveWallTeleOp(driveTrain, wheelOdometry);
                    moveWallTeleOp.setName("moveWallTeleop");
                }

            }
            if (moveWallTeleOp != null){
                moveWallTeleOp.updateCheckDone();
            }

            if(specimenRoundTripPressed) {
                if (wallToBarHangRoundTripTeleOp == null || wallToBarHangRoundTripTeleOp.getIsDone()){
                    hangPos += hangIncrement;
                    if (hangPos > (hangIncrement*6+100)) {
                        wallToBarHangRoundTripTeleOp = new WallToBarHangRoundTripTeleOp(driveTrain, wheelOdometry, outtake, hangPos, -30);
                        wallToBarHangRoundTripTeleOp.setName("wallToBarHangRoundTrip");
                    } else if (hangPos > (hangIncrement*2+100)) {
                        wallToBarHangRoundTripTeleOp = new WallToBarHangRoundTripTeleOp(driveTrain, wheelOdometry, outtake, hangPos, -15);
                        wallToBarHangRoundTripTeleOp.setName("wallToBarHangRoundTrip");
                    } else {
                        wallToBarHangRoundTripTeleOp = new WallToBarHangRoundTripTeleOp(driveTrain, wheelOdometry, outtake, hangPos, 0);
                        wallToBarHangRoundTripTeleOp.setName("wallToBarHangRoundTrip");
                    }
                }
            }

            if (wallToBarHangRoundTripTeleOp != null){
                wallToBarHangRoundTripTeleOp.updateCheckDone();
            }

            if (!isGamePadDriveJoystickZero()) {  //cacel action b/c of Manual control override
                if (angleLockTeleOp != null) {
                    angleLockTeleOp.setIsDone(true);
                }
                if (moveWallTeleOp != null) {
                    moveWallTeleOp.setIsDone(true);
                }
            }

            //HANG

            if (autoRobotHangAction != null){
                Log.d("teleop", "running auto robot hang action");
                autoRobotHangAction.updateCheckDone();
//                if (autoRobotHangAction.getIsDone()) {
//                    maintainLS = new MoveOuttakeLSAction(outtake);
//                }
            }

            if(hangPressed) {
                if (autoRobotHangAction == null || autoRobotHangAction.getIsDone()){
//                    maintainLS = null;
                    autoRobotHangAction = new AutoRobotHangAction(outtake);
                    autoRobotHangAction.setName("autoRobotHangAction");

                    needMaintainLs = false;

                    setLastLsAction(autoRobotHangAction);
                }
            }

            //INTAKE SAMPLE IN FUNNEL


            if(intakeFunnelEndToEndPressed) {
                if (intakeFunnelEndToEndAction == null || intakeFunnelEndToEndAction.getIsDone()){
                    intakeFunnelEndToEndAction = new FunnelEndToEndAction(intakeClaw, outtake);
                    intakeFunnelEndToEndAction.setName("intakeFunnelReady");
                }

            }
            if (intakeFunnelEndToEndAction != null){
                intakeFunnelEndToEndAction.updateCheckDone();
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
                    if(intakeClawPos == IntakeClaw.INTAKE_CLAW_CLOSE) {
                        intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    } else if (intakeClawPos == IntakeClaw.INTAKE_CLAW_OPEN){
                        intakeClawPos = IntakeClaw.INTAKE_CLAW_CLOSE;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    }
                }

                intakeClawOpenCloseWasPressed = false;
            }

            if (-intakeStickValue != 0) {
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
                intakeBigSweepPos -= 0.005;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            } else if (-sweepStickValue < -0.5 && intakeOverrideOn) {
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
                intakeSmallSweepPos += 0.025;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
                intakeClaw.getIntakeSmallSweepServo().setPosition(IntakeClaw.INTAKE_SMALL_SWEEP_VERTICAL_POS);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            } else if ((-sweepStickValue < -0.5) && !intakeOverrideOn) {
                intakeSmallSweepPos -= 0.025;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
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
                    sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw, IntakeClaw.INTAKE_SMALL_SWEEP_VERTICAL_POS);
                    Log.d("teleop", "made new sample intake  ready");
                    sampleIntakeReady.setName("sampleIntakeReady");
                }

            }
            if (sampleIntakeReady != null){
                Log.d("teleop", "running sample intake  ready");
                sampleIntakeReady.updateCheckDone();
            }

            if(intakePressed) {
                if (sampleIntakeAction == null || sampleIntakeAction.getIsDone()){
                    sampleIntakeAction = new SampleIntakeAction(intakeClaw);
                    sampleIntakeAction.setName("sampleIntakeAction");
                }

            }
            if (sampleIntakeAction != null){
                sampleIntakeAction.updateCheckDone();
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
                }

            }
            if (intakeTransferReady != null){
                intakeTransferReady.updateCheckDone();
            }

            if(transferPressed) {
                if (transferAction == null || transferAction.getIsDone()){
                    transferAction = new TransferAction(intakeClaw, outtake);
                    transferAction.setName("transferAction");
                }

            }
            if (transferAction != null){
                transferAction.updateCheckDone();
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

                setLastLsAction(moveLS);
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

                    setLastLsAction(outtakeTransferReady);
                }
            }

            //outtake hang ready
            if (specimenHangReadyPressed) {
                if (specimenHangReady == null || specimenHangReady.getIsDone()) {
                    specimenHangReady = new SpecimenHangReady(outtake);
                    specimenHangReady.setName("specimenHangReady");

                    setLastLsAction(specimenHangReady);
                }

            }

            if(specimenHangPressed) {
                if(specimenHang == null || specimenHang.getIsDone()){
                   specimenHang = new SpecimenHang(outtake);
                   specimenHang.setName("specimenHang");

                    setLastLsAction(specimenHang);
                }
            }

            if(basketReadyPressed) {
                if (basketReadyAction == null || basketReadyAction.getIsDone()){
                    basketReadyAction = new BasketReadyAction(outtake);
                    basketReadyAction.setName("basketReadyAction");

                    setLastLsAction(basketReadyAction);
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
                        outtake.getOuttakeClaw().setPosition(outtakeClawPos);
                    }

                    outtakeClawOpenCloseWasPressed = false;
                }
            }

            if (outtakePivotStickValue > 0.5) {
                outtake.getOuttakePivotServo().setPosition(Outtake.OUTTAKE_PIVOT_BASKET_POS + 0.12);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            } else if (outtakePivotStickValue < -0.5) {
                outtake.getOuttakePivotServo().setPosition(Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            }

            if(sampleEndToEndSequencePressed) {
                if(sampleEndToEndSequence == null || sampleEndToEndSequence.getIsDone()){
                    sampleEndToEndSequence = new SampleEndToEndSequence(intakeClaw, outtake);
                    sampleEndToEndSequence.setName("sampleEndToEndSequence");

                    setLastLsAction(sampleEndToEndSequence);
                }
            }

            if(specimenEndToEndSequencePressed) {
                Log.d("backButton", " " + specimenEndToEndSequencePressed);
                if(specimenEndToEndSequence == null || specimenEndToEndSequence.getIsDone()){
                    specimenEndToEndSequence = new SampleEndToEndSequence(intakeClaw, outtake, Outtake.LS_DOWN_POS);
                    specimenEndToEndSequence.setName("specimenEndToEndSequence");
                    Log.d("backButton", "Creating sequence " + specimenEndToEndSequence);

                    setLastLsAction(specimenEndToEndSequence);
                }
            }

            // Capture pictures from webcam every 500 milliseconds if holding dpad right with gamepad 1
            //DO NOT USE ALL HOLD FOR TOO LONG it will take up to much space.
            if (gamepad1.dpad_right) {
                cameraCapture.capture();
            }


//            if(maintainLS != null && MoveOuttakeLSAction.getNeedMaintenance()) {
//                maintainLS.setIsDone(true);
//                maintainLS.update();
//            }

            wheelOdometry.updatePosition();

            if(lastLSAction != null) {
                lastLSAction.updateCheckDone();
            }

            if(needMaintainLs) {
                if (lastLSAction == null || lastLSAction.getIsDone()) {
                    maintainLS.setIsDone(false);
                    maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
                    maintainLS.updateCheckDone();
                }
            }

            telemetry.addData("odometry: ", wheelOdometry.getCurrentPosition().toString());
            telemetry.addData("big sweep pos: ", intakeBigSweepPos);
            telemetry.addData("small sweep pos: ", intakeSmallSweepPos);
            telemetry.update();
            Log.d("teleopforauto", "odometry " + wheelOdometry.getCurrentPosition().toString());
            Log.d("teleopforauto", "big sweep " + intakeClaw.getIntakeBigSweepServo().getPosition());
            Log.d("teleopforauto", "small sweep " + intakeClaw.getIntakeSmallSweepServo().getPosition());

        }
    }

    private void setLastLsAction(Action action) {
        if(lastLSAction != null) {
            lastLSAction.setIsDone(true);
        }
        lastLSAction = action;
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

