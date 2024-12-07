package com.kalipsorobotics.intoTheDeep;


import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeDownReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedAutoBasket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoBasket = new KActionSet();
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        // Target can always be 0 because Hung said so
        MoveOuttakeLSAction maintainLS = new MoveOuttakeLSAction(outtake, 0);
//                MoveLSAction.globalLinearSlideMaintainTicks);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intake, outtake);
        initAuto.setName("initAuto");

        int outtakeXPos = -175;
        int outtakeYPos = 1075;

        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(300);
        waitAtStart.setName("waitAtStart");
        redAutoBasket.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-785, -300, 0);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        redAutoBasket.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        redAutoBasket.addAction(specimenHangReady1);

        MoveOuttakeLSAction lowerSlidesHalf1 = new MoveOuttakeLSAction(outtake, 200);
        lowerSlidesHalf1.setName("lowerSlidesHalf1");
        lowerSlidesHalf1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
        redAutoBasket.addAction(lowerSlidesHalf1);

        WaitAction waitAfterHang = new WaitAction(500);
        waitAfterHang.setName("waitAfterHang");
        waitAfterHang.setDependentActions(lowerSlidesHalf1);
        redAutoBasket.addAction(waitAfterHang);

        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw.setName("openClaw");
        openClaw.setDependentActions(waitAfterHang);
        redAutoBasket.addAction(openClaw);

        OuttakeDownReady outtakeDownReady1 = new OuttakeDownReady(outtake);
        outtakeDownReady1.setName("outtakeDownReady1");
        outtakeDownReady1.setDependentActions(openClaw);
        redAutoBasket.addAction(outtakeDownReady1);
        //===============end of first specimen===============



        //================begin of first basket====================
        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        moveToSample1.setDependentActions(waitAfterHang);
        //bar to sample 1
        moveToSample1.addPoint(-620, 820, 180);
        redAutoBasket.addAction(moveToSample1);

        //TODO INTAKE ACTION

        PurePursuitAction moveToBasket1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket1.setName("moveToBasket1");
        moveToBasket1.setDependentActions(moveToSample1);
        //move sample 1 to basket
        moveToBasket1.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket1);

        BasketReadyAction basketReady1 = new BasketReadyAction(outtake);
        basketReady1.setName("basketReady1");
        basketReady1.setDependentActions(moveToBasket1);
        redAutoBasket.addAction(basketReady1);

        KServoAutoAction outtakePivotActionOut1 = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_BASKET_POS);
        outtakePivotActionOut1.setName("outtakePivotActionOut1");
        outtakePivotActionOut1.setDependentActions(basketReady1);
        redAutoBasket.addAction(outtakePivotActionOut1);

        KServoAutoAction openClaw1 = new KServoAutoAction(outtake.getOuttakeClawServo(), OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw1.setName("openClaw1");
        openClaw1.setDependentActions(outtakePivotActionOut1);
        redAutoBasket.addAction(openClaw1);

        OuttakeDownReady outtakeDownReady2 = new OuttakeDownReady(outtake);
        outtakeDownReady2.setName("outtakeDownReady2");
        outtakeDownReady2.setDependentActions(openClaw1);
        redAutoBasket.addAction(outtakeDownReady2);
        //===============end of first basket===============



        //===============start of second basket===============
        PurePursuitAction moveToSample2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample2.setName("moveToSample2");
        moveToSample2.setDependentActions(outtakeDownReady2);
        //move basket to sample 2
        moveToSample2.addPoint(-620, 950, 180);
        redAutoBasket.addAction(moveToSample2);

        //TODO INTAKE ACTION

        PurePursuitAction moveToBasket2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket2.setName("moveToBasket2");
        moveToBasket2.setDependentActions(moveToSample2/*TODO intake*/);
        //move sample 2 to basket
        moveToBasket2.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket2);

        BasketReadyAction basketReady2 = new BasketReadyAction(outtake);
        basketReady2.setName("basketReady2");
        basketReady2.setDependentActions(moveToBasket2);
        redAutoBasket.addAction(basketReady2);

        KServoAutoAction outtakePivotActionOut2 = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_BASKET_POS);
        outtakePivotActionOut2.setName("outtakePivotActionOut2");
        outtakePivotActionOut2.setDependentActions(basketReady2);
        redAutoBasket.addAction(outtakePivotActionOut2);

        KServoAutoAction openClaw2 = new KServoAutoAction(outtake.getOuttakeClawServo(), OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw2.setName("openClaw2");
        openClaw2.setDependentActions(outtakePivotActionOut2);
        redAutoBasket.addAction(openClaw2);

        OuttakeDownReady outtakeDownReady3 = new OuttakeDownReady(outtake);
        outtakeDownReady3.setName("outtakeDownReady3");
        outtakeDownReady3.setDependentActions(openClaw2);
        redAutoBasket.addAction(outtakeDownReady3);
        //===============end of second basket===============



        //===============start of third basket===============
        PurePursuitAction moveToSample3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample3.setName("moveToSample3");
        moveToSample3.setDependentActions(moveToBasket2);
        //move basket to sample 3
        moveToSample3.addPoint(-930, 1010, 90);
        redAutoBasket.addAction(moveToSample3);

        //TODO INTAKE ACTION

        PurePursuitAction moveToBasket3 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToBasket3.setName("moveToBasket3");
        moveToBasket3.setDependentActions(moveToSample3/*TODO intake*/);
        //move sample 1 to basket
        moveToBasket3.addPoint(outtakeXPos, outtakeYPos, -135);
        redAutoBasket.addAction(moveToBasket3);

        BasketReadyAction basketReady3 = new BasketReadyAction(outtake);
        basketReady3.setName("basketReady3");
        basketReady3.setDependentActions(moveToBasket3);
        redAutoBasket.addAction(basketReady3);

        KServoAutoAction outtakePivotActionOut3 = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_BASKET_POS);
        outtakePivotActionOut3.setName("outtakePivotActionOut3");
        outtakePivotActionOut3.setDependentActions(basketReady3);
        redAutoBasket.addAction(outtakePivotActionOut3);

        KServoAutoAction openClaw3 = new KServoAutoAction(outtake.getOuttakeClawServo(), OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw3.setName("openClaw3");
        openClaw3.setDependentActions(outtakePivotActionOut3);
        redAutoBasket.addAction(openClaw3);

        OuttakeDownReady outtakeDownReady4 = new OuttakeDownReady(outtake);
        outtakeDownReady4.setName("outtakeDownReady4");
        outtakeDownReady4.setDependentActions(openClaw3);
        redAutoBasket.addAction(outtakeDownReady4);
//
//        moveToSpecimenBar.addPoint(0, 0, 0);
//        moveToSpecimenBar.addPoint(-740, -300, 0);


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