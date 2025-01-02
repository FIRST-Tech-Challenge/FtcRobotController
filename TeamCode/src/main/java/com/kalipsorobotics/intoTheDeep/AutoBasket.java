package com.kalipsorobotics.intoTheDeep;


import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
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
public class AutoBasket extends LinearOpMode {

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
        MoveOuttakeLSAction maintainLS = new MoveOuttakeLSAction(outtake, 0);
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        final int INTAKE_SAMPLE_X = -660;
        int outtakeXPos = -225;
        int outtakeYPos = 1020;

        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(300);
        waitAtStart.setName("waitAtStart");
        redAutoBasket.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, -350, 0);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        redAutoBasket.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        redAutoBasket.addAction(specimenHangReady1);

        SpecimenHang specimenHang1 = new SpecimenHang(outtake);
        specimenHang1.setName("specimenHang1");
        specimenHang1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
        redAutoBasket.addAction(specimenHang1);
        //===============end of first specimen===============



        //================begin of first basket====================
        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        moveToSample1.setDependentActions(specimenHang1);
        //bar to sample 1
        moveToSample1.addPoint(-420, 200, 90);
        moveToSample1.addPoint(INTAKE_SAMPLE_X, 840, 180);
        redAutoBasket.addAction(moveToSample1);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(specimenHang1);
        redAutoBasket.addAction(outtakeTransferReady);

        SampleIntakeReady sampleIntakeReady1 = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_IN_POS, intakeClaw);
        sampleIntakeReady1.setName("sampleIntakeReady1");
        sampleIntakeReady1.setDependentActions(specimenHang1);
        redAutoBasket.addAction(sampleIntakeReady1);

        SampleIntakeAction sampleIntakeAction1 = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction1.setName("sampleIntakeAction1");
        sampleIntakeAction1.setDependentActions(sampleIntakeReady1, moveToSample1);
        redAutoBasket.addAction(sampleIntakeAction1);

        IntakeTransferReady intakeTransferReady1 = new IntakeTransferReady(intakeClaw);
        intakeTransferReady1.setName("intakeTransferReady1");
        intakeTransferReady1.setDependentActions(sampleIntakeAction1);
        redAutoBasket.addAction(intakeTransferReady1);

        TransferAction transferAction1 = new TransferAction(intakeClaw, outtake);
        transferAction1.setName("transferAction1");
        transferAction1.setDependentActions(intakeTransferReady1);
        redAutoBasket.addAction(transferAction1);

        //TODO INTAKE ACTION

//        IntakeReadyAction intakeReady1 = new IntakeReadyAction(10, intake);
//        intakeReady1.setName("intakeReady1");
//        intakeReady1.setDependentActions(moveToSample1);
//        redAutoBasket.addAction(intakeReady1);

//        IntakeAction intake1 = new IntakeAction(intake);
//        intake1.setName("intake1");
//        intake1.setDependentActions(intakeReady1);
//        redAutoBasket.addAction(intake1);

        PurePursuitAction moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setName("moveToBasket1");
        moveToBasket1.setDependentActions(sampleIntakeAction1);
        //move sample 1 to basket
        moveToBasket1.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket1);

        BasketReadyAction basketReady1 = new BasketReadyAction(outtake);
        basketReady1.setName("basketReady1");
        basketReady1.setDependentActions(moveToBasket1, transferAction1);
        redAutoBasket.addAction(basketReady1);

        WaitAction waitAction1 = new WaitAction(100);
        waitAction1.setName("waitAction1");
        waitAction1.setDependentActions(basketReady1);
        redAutoBasket.addAction(waitAction1);
//
//        KServoAutoAction outtakePivotActionOut1 = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                Outtake.OUTTAKE_PIVOT_BASKET_POS);
//        outtakePivotActionOut1.setName("outtakePivotActionOut1");
//        outtakePivotActionOut1.setDependentActions(basketReady1);
//        redAutoBasket.addAction(outtakePivotActionOut1);

        KServoAutoAction openClaw1 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw1.setName("openClaw1");
        openClaw1.setDependentActions(basketReady1);
        redAutoBasket.addAction(openClaw1);
        //===============end of first basket===============



        //===============start of second basket===============
        PurePursuitAction moveOutBasket1 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket1.setName("moveOutBasket1");
        moveOutBasket1.setDependentActions(openClaw1, waitAction1);
        moveOutBasket1.addPoint(outtakeXPos - 100, outtakeYPos - 100, -135);
        redAutoBasket.addAction(moveOutBasket1);
//
        OuttakeTransferReady outtakeTransferReady1 = new OuttakeTransferReady(outtake);
        outtakeTransferReady1.setName("outtakeTransferReady1");
        outtakeTransferReady1.setDependentActions(moveOutBasket1);
        redAutoBasket.addAction(outtakeTransferReady1);

        PurePursuitAction moveToSample2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample2.setName("moveToSample2");
        moveToSample2.setDependentActions(moveOutBasket1);
        //move basket to sample 2
        moveToSample2.addPoint(INTAKE_SAMPLE_X+25, 1100, 180);
        redAutoBasket.addAction(moveToSample2);

        SampleIntakeReady sampleIntakeReady2 = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_IN_POS, intakeClaw);
        sampleIntakeReady2.setName("sampleIntakeReady2");
        sampleIntakeReady2.setDependentActions(moveOutBasket1);
        redAutoBasket.addAction(sampleIntakeReady2);

        SampleIntakeAction sampleIntakeAction2 = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction2.setName("sampleIntakeAction2");
        sampleIntakeAction2.setDependentActions(sampleIntakeReady2, moveToSample2);
        redAutoBasket.addAction(sampleIntakeAction2);

        IntakeTransferReady intakeTransferReady2 = new IntakeTransferReady(intakeClaw);
        intakeTransferReady2.setName("intakeTransferReady2");
        intakeTransferReady2.setDependentActions(sampleIntakeAction2);
        redAutoBasket.addAction(intakeTransferReady2);

        TransferAction transferAction2 = new TransferAction(intakeClaw, outtake);
        transferAction2.setName("transferAction2");
        transferAction2.setDependentActions(intakeTransferReady2);
        redAutoBasket.addAction(transferAction2);

        //TODO INTAKE ACTION

//        IntakeReadyAction intakeReady2 = new IntakeReadyAction(10, intake);
//        intakeReady2.setName("intakeReady2");
//        intakeReady2.setDependentActions(moveToSample2);
//        redAutoBasket.addAction(intakeReady2);
//
//        IntakeAction intake2 = new IntakeAction(intake);
//        intake2.setName("intake2");
//        intake2.setDependentActions(intakeReady2);
//        redAutoBasket.addAction(intake2);

        PurePursuitAction moveToBasket2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket2.setName("moveToBasket2");
        moveToBasket2.setDependentActions(sampleIntakeAction2);
        //move sample 2 to basket
        moveToBasket2.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket2);

        BasketReadyAction basketReady2 = new BasketReadyAction(outtake);
        basketReady2.setName("basketReady2");
        basketReady2.setDependentActions(moveToBasket2, transferAction2);
        redAutoBasket.addAction(basketReady2);

        WaitAction waitAction2 = new WaitAction(100);
        waitAction2.setName("waitAction2");
        waitAction2.setDependentActions(basketReady2);
        redAutoBasket.addAction(waitAction2);
//
//        KServoAutoAction outtakePivotActionOut1 = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                Outtake.OUTTAKE_PIVOT_BASKET_POS);
//        outtakePivotActionOut1.setName("outtakePivotActionOut1");
//        outtakePivotActionOut1.setDependentActions(basketReady1);
//        redAutoBasket.addAction(outtakePivotActionOut1);

        KServoAutoAction openClaw2 = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw2.setName("openClaw2");
        openClaw2.setDependentActions(basketReady2);
        redAutoBasket.addAction(openClaw2);
        //===============end of second basket===============



        //===============start of third basket===============
        PurePursuitAction moveOutBasket2 = new PurePursuitAction(driveTrain,wheelOdometry);
        moveOutBasket2.setName("moveOutBasket2");
        moveOutBasket2.setDependentActions(openClaw2, waitAction2);
        moveOutBasket2.addPoint(outtakeXPos - 100, outtakeYPos - 100, -135);
        redAutoBasket.addAction(moveOutBasket2);
//
        OuttakeTransferReady outtakeTransferReady2 = new OuttakeTransferReady(outtake);
        outtakeTransferReady2.setName("outtakeTransferReady2");
        outtakeTransferReady2.setDependentActions(moveOutBasket2);
        redAutoBasket.addAction(outtakeTransferReady2);

        PurePursuitAction moveToSample3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample3.setName("moveToSample3");
        moveToSample3.setDependentActions(moveOutBasket2);
        //move basket to sample 2
        moveToSample3.addPoint(-405, 1064.35, 180-27.6);
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
        transferAction3.setDependentActions(intakeTransferReady3);
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

        MoveOuttakeLSAction lsTouchBar = new MoveOuttakeLSAction(outtake, Outtake.LS_SPECIMAN_PARK_MM);
        lsTouchBar.setName("lsTouchBar");
        lsTouchBar.setDependentActions(pivotOuttakeHalfwayToBar, moveOutBasket3);
        redAutoBasket.addAction(lsTouchBar);

        PurePursuitAction park = new PurePursuitAction(driveTrain, wheelOdometry);
        park.setName("park");
        park.setDependentActions(lsTouchBar, moveOutBasket3);
        park.addPoint(-1325, 610, 45);
        park.addPoint(-1425, 260, 90);
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
            maintainLS.update();

            redAutoBasket.updateCheckDone();


        }

    }
}