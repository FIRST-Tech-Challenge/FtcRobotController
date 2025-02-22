package com.kalipsorobotics.intoTheDeep;


import android.util.Log;

import com.kalipsorobotics.actions.intake.IntakeTransferThirdSampleReady;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.FirstWallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.SampleToBasketFunnelRoundTrip;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name = "AutoBasket 1 SPECIMEN 3 BASKET 3+1")
public class AutoBasketFunnel extends LinearOpMode {

    protected boolean isFirstMoveSpecimen = true;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoBasket = new KActionSet();

        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        Outtake.setInstanceNull();
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        IntakeClaw.setInstanceNull();
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        SharedData.resetOdometryPosition();


        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        final int INTAKE_SAMPLE_X = -590-300;

//        int outtakeXPos = -190;
//        int outtakeYPos = 1020;

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMs());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoBasket.addAction(delayBeforeStart);

        //================begin of first specimen====================
        PurePursuitAction moveOutSpecimen = null;
        IntakeFunnelReady intakeFunnelReady1 = null;
        PurePursuitAction moveToBasket1 = null;
        PurePursuitAction moveOutBasket1 = null;
        if (isFirstMoveSpecimen) {

            FirstWallToBarHangAction firstWallToBarHangAction = new FirstWallToBarHangAction(driveTrain, wheelOdometry, outtake, -290);
            firstWallToBarHangAction.setName("wallToBarHangAction");
            firstWallToBarHangAction.setDependentActions(delayBeforeStart);
            redAutoBasket.addAction(firstWallToBarHangAction);

            moveOutSpecimen = new PurePursuitAction(driveTrain, wheelOdometry);
            moveOutSpecimen.setName("moveOutSpecimen");
            moveOutSpecimen.setFinalSearchRadius(75);
            moveOutSpecimen.setDependentActions(firstWallToBarHangAction);
            moveOutSpecimen.addPoint(-500, 775, 180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE); // y = -250 (midpt)
            redAutoBasket.addAction(moveOutSpecimen);

        } else {

            moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
            moveToBasket1.setName("moveToBasket3");
            //move sample 3 to basket
            moveToBasket1.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS - 100,
                    SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS - 125, -135,
                    PurePursuitAction.P_XY_FAST,
                    PurePursuitAction.P_ANGLE_FAST);
            moveToBasket1.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS, SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS,
                    -135, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
            redAutoBasket.addAction(moveToBasket1);

            WaitAction waitForPurePure = new WaitAction(150);
            waitForPurePure.setName("waitForPurePure");
            redAutoBasket.addAction(waitForPurePure);

            BasketReadyAction basketReady3 = new BasketReadyAction(outtake, Outtake.OUTTAKE_PIVOT_AUTO_BASKET_POS);
            basketReady3.setName("basketReady3");
            basketReady3.setDependentActions(waitForPurePure);
            redAutoBasket.addAction(basketReady3);

            KServoAutoAction openClaw3 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
            openClaw3.setName("openClaw3");
            openClaw3.setDependentActions(basketReady3, moveToBasket1);
            redAutoBasket.addAction(openClaw3);

            moveOutBasket1 = new PurePursuitAction(driveTrain,wheelOdometry);
            moveOutBasket1.setName("moveOutBasket3");
            moveOutBasket1.setDependentActions(openClaw3);
            moveOutBasket1.setFinalSearchRadius(50);
            moveOutBasket1.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS - 150,
                    SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS - 150, -135,
                    PurePursuitAction.P_XY_SLOW,
                    PurePursuitAction.P_ANGLE_SLOW);
            redAutoBasket.addAction(moveOutBasket1);

        }

        intakeFunnelReady1 = new IntakeFunnelReady(intakeClaw, outtake, false);
        intakeFunnelReady1.setName("intakeFunnelReady1");
        intakeFunnelReady1.setDependentActions(delayBeforeStart);
        redAutoBasket.addAction(intakeFunnelReady1);

        //=================end of first specimen=================


        //================begin of first basket====================
        SampleToBasketFunnelRoundTrip sampleToBasketFunnelRoundTrip1 = new SampleToBasketFunnelRoundTrip(driveTrain,
                wheelOdometry, outtake, intakeClaw, 850);
        sampleToBasketFunnelRoundTrip1.setName("sampleToBasketFunnelRoundTrip1");
        sampleToBasketFunnelRoundTrip1.setDependentActions(moveOutSpecimen, intakeFunnelReady1, moveOutBasket1);
        redAutoBasket.addAction(sampleToBasketFunnelRoundTrip1);
        //===============end of first basket===============


        //===============start of second basket===============
        SampleToBasketFunnelRoundTrip sampleToBasketFunnelRoundTrip2 = new SampleToBasketFunnelRoundTrip(driveTrain,
                wheelOdometry, outtake, intakeClaw, 1100, -75);
        sampleToBasketFunnelRoundTrip2.setName("sampleToBasketFunnelRoundTrip2");
        sampleToBasketFunnelRoundTrip2.setDependentActions(sampleToBasketFunnelRoundTrip1);
        redAutoBasket.addAction(sampleToBasketFunnelRoundTrip2);
        //===============end of second basket===============


        //===============start of third basket===============
        OuttakeTransferReady outtakeTransferReady2 = new OuttakeTransferReady(outtake);
        outtakeTransferReady2.setName("outtakeTransferReady2");
        outtakeTransferReady2.setDependentActions(sampleToBasketFunnelRoundTrip2);
        redAutoBasket.addAction(outtakeTransferReady2);

        PurePursuitAction moveToSample3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample3.setName("moveToSample3");
        moveToSample3.setDependentActions(sampleToBasketFunnelRoundTrip2);
        //move basket to sample 3
//        moveToSample3.addPoint(-440, 1030, 180-29.6);
        moveToSample3.addPoint(INTAKE_SAMPLE_X-100, 750, 90, PurePursuitAction.P_XY_SLOW,
                PurePursuitAction.P_ANGLE); //x = INtAKE_SAMPLE_X - 80, y = 760
//        moveToSample3.setMaxCheckDoneCounter(15);
        redAutoBasket.addAction(moveToSample3);

        //TODO INTAKE ACTION

//        SampleIntakeReady sampleIntakeReady3 = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw, IntakeClaw.INTAKE_SMALL_SWEEP_THIRD_SAMPLE_BASKET_GRAB_POS);
//        sampleIntakeReady3.setName("sampleIntakeReady3");
//        sampleIntakeReady3.setDependentActions(sampleToBasketFunnelRoundTrip2);
//        redAutoBasket.addAction(sampleIntakeReady3);

        SampleIntakeReady sampleIntakeReady3 = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw, IntakeClaw.INTAKE_SMALL_SWEEP_VERTICAL_POS);
        sampleIntakeReady3.setName("sampleIntakeReady3");
        sampleIntakeReady3.setDependentActions(sampleToBasketFunnelRoundTrip2, moveToSample3);
        redAutoBasket.addAction(sampleIntakeReady3);

        WaitAction waitAction3 = new WaitAction(300); // Make sure linkage fully extends (sample 3)
        waitAction3.setName("waitAction");
        waitAction3.setDependentActions(sampleIntakeReady3);
        redAutoBasket.addAction(waitAction3);

        SampleIntakeAction sampleIntakeAction3 = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction3.setName("sampleIntakeAction3");
        sampleIntakeAction3.setDependentActions(sampleIntakeReady3, moveToSample3, sampleIntakeReady3, waitAction3);
        redAutoBasket.addAction(sampleIntakeAction3);

        IntakeTransferThirdSampleReady intakeTransferReady3 = new IntakeTransferThirdSampleReady(intakeClaw);
        intakeTransferReady3.setName("intakeTransferReady3");
        intakeTransferReady3.setDependentActions(sampleIntakeAction3);
        redAutoBasket.addAction(intakeTransferReady3);

        TransferAction transferAction3 = new TransferAction(intakeClaw, outtake);
        transferAction3.setName("transferAction3");
        transferAction3.setDependentActions(intakeTransferReady3, outtakeTransferReady2);
        redAutoBasket.addAction(transferAction3);
//
//        IntakeReadyAction intakeReady3 = new IntakeReadyAction(10, intake);
//        intakeReady3.setName("intakeReady3");
//        intakeReady3.setDependentActions(moveToSample3);
//        redAutoBasket.addAction(intakeReady3);
//
//        IntakeAction intake3 = new IntakeAction(intake);
//        intake3.setName("intake3");
//        intake3.setDependentActions(intakeReady3);
//        redAutoBasket.addAction(intake3);

        PurePursuitAction moveToBasket3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket3.setName("moveToBasket3");
        moveToBasket3.setDependentActions(sampleIntakeAction3, transferAction3);
        //move sample 3 to basket
        moveToBasket3.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS - 250,
                SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS - 125, -135,
                PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE);
        moveToBasket3.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS, SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS,
                -135, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
        redAutoBasket.addAction(moveToBasket3);

        BasketReadyAction basketReady3 = new BasketReadyAction(outtake, Outtake.OUTTAKE_PIVOT_AUTO_BASKET_POS);
        basketReady3.setName("basketReady3");
        basketReady3.setDependentActions(moveToBasket3, transferAction3);
        redAutoBasket.addAction(basketReady3);

        WaitAction waitForPivot = new WaitAction(100);
        waitForPivot.setDependentActions(basketReady3);
        redAutoBasket.addAction(waitForPivot);

        KServoAutoAction openClaw3 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw3.setName("openClaw3");
        openClaw3.setDependentActions(waitForPivot);
        redAutoBasket.addAction(openClaw3);

        PurePursuitAction moveOutBasket3 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket3.setName("moveOutBasket3");
        moveOutBasket3.setDependentActions(openClaw3);
        moveOutBasket3.setFinalSearchRadius(50);
        moveOutBasket3.addPoint(SampleToBasketFunnelRoundTrip.OUTTAKE_X_POS - 150,
                SampleToBasketFunnelRoundTrip.OUTTAKE_Y_POS - 150, -135,
                PurePursuitAction.P_XY_SLOW,
                PurePursuitAction.P_ANGLE_SLOW);
        redAutoBasket.addAction(moveOutBasket3);

        KServoAutoAction pivotOuttakeHalfwayToBar = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_PARKING_READY_POS);
        pivotOuttakeHalfwayToBar.setName("pivotOuttakeHalfwayToBar");
        pivotOuttakeHalfwayToBar.setDependentActions(openClaw3);
        redAutoBasket.addAction(pivotOuttakeHalfwayToBar);

        MoveLSAction lsTouchBar = new MoveLSAction(outtake, Outtake.LS_SPECIMEN_PARK_MM);
        lsTouchBar.setName("lsTouchBar");
        lsTouchBar.setDependentActions(moveOutBasket3);
        redAutoBasket.addAction(lsTouchBar);

        PurePursuitAction park = new PurePursuitAction(driveTrain, wheelOdometry);
        park.setName("park");
        park.setDependentActions(moveOutBasket3);
        park.setFinalSearchRadius(40);
        park.addPoint(-1225, 700, 45, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); // 610
        park.addPoint(-1325, 190, 90, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);
        redAutoBasket.addAction(park);

        KServoAutoAction pivotOuttakeToBar = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_TOUCH_BAR_POS);
        pivotOuttakeToBar.setName("pivotOuttakeToBar");
        pivotOuttakeToBar.setDependentActions(lsTouchBar, pivotOuttakeHalfwayToBar, park);
        redAutoBasket.addAction(pivotOuttakeToBar);

        //bar to sample 1

        //intake sample 1

        //move sample to basket sample 1

        //outtake sample 1

        //move basket to sample 2

        //intake sample 2

        //move sample to basket sample 2

        //outtake sample 2

        //move basket to third sample

        //move sample to basket 3

        //outtake sample 3

        telemetry.addLine("init finished");
        telemetry.update();

        ExecutorService executorService = Executors.newSingleThreadExecutor();


        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);

        while (opModeIsActive()) {

            //wheelOdometry.updatePosition();

            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();

            redAutoBasket.updateCheckDone();

        }
        Log.d("executor service", "before shutdown" + SharedData.getOdometryPosition());
        OpModeUtilities.shutdownExecutorService(executorService);
        Log.d("executor service",
                "after shutdown" + SharedData.getOdometryPosition() + "is shutdown " + executorService.isShutdown() + "is terminated " + executorService.isTerminated());

    }
}