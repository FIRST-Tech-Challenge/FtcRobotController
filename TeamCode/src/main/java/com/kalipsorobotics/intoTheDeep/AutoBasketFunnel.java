package com.kalipsorobotics.intoTheDeep;


import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.SampleToBasketFunnelRoundTrip;
import com.kalipsorobotics.actions.intake.IntakeFunnelAction;
import com.kalipsorobotics.actions.intake.IntakeFunnelReady;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoBasketFunnel extends LinearOpMode {

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
        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        final int INTAKE_SAMPLE_X = -590-300;

        int outtakeXPos = -190;
        int outtakeYPos = 1020;

        //================begin of first specimen====================
        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, -350);
        wallToBarHangAction.setName("wallToBarHangAction");
        redAutoBasket.addAction(wallToBarHangAction);
        //===============end of first specimen===============



        //================begin of first basket====================
        SampleToBasketFunnelRoundTrip sampleToBasketFunnelRoundTrip1 = new SampleToBasketFunnelRoundTrip(driveTrain, wheelOdometry, outtake, intakeClaw, 840);
        sampleToBasketFunnelRoundTrip1.setName("sampleToBasketFunnelRoundTrip1");
        sampleToBasketFunnelRoundTrip1.setDependentActions(wallToBarHangAction);
        redAutoBasket.addAction(sampleToBasketFunnelRoundTrip1);
        //===============end of first basket===============



        //===============start of second basket===============
        SampleToBasketFunnelRoundTrip sampleToBasketFunnelRoundTrip2 = new SampleToBasketFunnelRoundTrip(driveTrain, wheelOdometry, outtake, intakeClaw, 1100);
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
        //move basket to sample 2
        moveToSample3.addPoint(-350, 1054.35, 180-27.6);
        redAutoBasket.addAction(moveToSample3);

        //TODO INTAKE ACTION

        SampleIntakeReady sampleIntakeReady3 = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw, IntakeClaw.INTAKE_SMALL_SWEEP_THIRD_SAMPLE_BASKET_GRAB_POS);
        sampleIntakeReady3.setName("sampleIntakeReady3");
        sampleIntakeReady3.setDependentActions(moveToSample3);
        redAutoBasket.addAction(sampleIntakeReady3);

        WaitAction waitAction3 = new WaitAction(500);
        waitAction3.setName("waitAction");
        waitAction3.setDependentActions(sampleIntakeReady3);
        redAutoBasket.addAction(waitAction3);

        SampleIntakeAction sampleIntakeAction3 = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction3.setName("sampleIntakeAction3");
        sampleIntakeAction3.setDependentActions(sampleIntakeReady3, moveToSample3, sampleIntakeReady3, waitAction3);
        redAutoBasket.addAction(sampleIntakeAction3);

        IntakeTransferReady intakeTransferReady3 = new IntakeTransferReady(intakeClaw);
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
        moveToBasket3.setDependentActions(sampleIntakeAction3);
        //move sample 3 to basket
        moveToBasket3.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket3);

        BasketReadyAction basketReady3 = new BasketReadyAction(outtake);
        basketReady3.setName("basketReady3");
        basketReady3.setDependentActions(moveToBasket3, transferAction3);
        redAutoBasket.addAction(basketReady3);

        WaitAction waitAction4 = new WaitAction(100);
        waitAction4.setName("waitAction4");
        waitAction4.setDependentActions(basketReady3);
        redAutoBasket.addAction(waitAction4);
//
//        KServoAutoAction outtakePivotActionOut1 = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                Outtake.OUTTAKE_PIVOT_BASKET_POS);
//        outtakePivotActionOut1.setName("outtakePivotActionOut1");
//        outtakePivotActionOut1.setDependentActions(basketReady1);
//        redAutoBasket.addAction(outtakePivotActionOut1);

        KServoAutoAction openClaw3 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw3.setName("openClaw3");
        openClaw3.setDependentActions(basketReady3);
        redAutoBasket.addAction(openClaw3);

        PurePursuitAction moveOutBasket3 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket3.setName("moveOutBasket3");
        moveOutBasket3.setDependentActions(openClaw3, waitAction4);
        moveOutBasket3.addPoint(outtakeXPos - 100, outtakeYPos - 100, -135);
        redAutoBasket.addAction(moveOutBasket3);

//        moveToSpecimenBar.addPoint(0, 0, 0);
//        moveToSpecimenBar.addPoint(-740, -300, 0);

        KServoAutoAction pivotOuttakeHalfwayToBar = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        pivotOuttakeHalfwayToBar.setName("pivotOuttakeHalfwayToBar");
        pivotOuttakeHalfwayToBar.setDependentActions(openClaw3, waitAction4);
        redAutoBasket.addAction(pivotOuttakeHalfwayToBar);

        MoveLSAction lsTouchBar = new MoveLSAction(outtake, Outtake.LS_SPECIMAN_PARK_MM);
        lsTouchBar.setName("lsTouchBar");
        lsTouchBar.setDependentActions(pivotOuttakeHalfwayToBar, moveOutBasket3);
        redAutoBasket.addAction(lsTouchBar);

        PurePursuitAction park = new PurePursuitAction(driveTrain, wheelOdometry);
        park.setName("park");
        park.setDependentActions(lsTouchBar, moveOutBasket3);
        park.addPoint(-1225, 610, 45);
        park.addPoint(-1325, 260, 90);
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

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();

            redAutoBasket.updateCheckDone();


        }

    }
}