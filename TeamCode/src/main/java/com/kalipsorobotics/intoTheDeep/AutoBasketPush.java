package com.kalipsorobotics.intoTheDeep;


import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoBasketPush extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoBasket = new KActionSet();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        Outtake outtake = Outtake.getInstance(opModeUtilities);
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        // Target can always be 0 because Hung said so
        MoveOuttakeLSAction maintainLS = new MoveOuttakeLSAction(outtake, 0);
//                MoveLSAction.globalLinearSlideMaintainTicks);
        maintainLS.setName("maintainLS");

        int basketOuttakeXPos = -175;
        int basketOuttakeYPos = 1075;

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");


        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(5000);
        waitAtStart.setName("waitAtStart");
        redAutoBasket.addAction(waitAtStart);
//
//        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
//        moveToSpecimenBar.setName("moveToSpecimenBar");
//        moveToSpecimenBar.addPoint(0, 0, 0);
//        moveToSpecimenBar.addPoint(-785, -350, 0);
//        moveToSpecimenBar.setMaxTimeOutMS(3000);
//        moveToSpecimenBar.setDependentActions(waitAtStart);
//        redAutoBasket.addAction(moveToSpecimenBar);
//
//        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
//        specimenHangReady1.setName("hangSpecimenReady1");
//        redAutoBasket.addAction(specimenHangReady1);
//
//        MoveOuttakeLSAction lowerSlidesHalf1 = new MoveOuttakeLSAction(outtake, 200);
//        lowerSlidesHalf1.setName("lowerSlidesHalf1");
//        lowerSlidesHalf1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
//        redAutoBasket.addAction(lowerSlidesHalf1);
//
//        WaitAction waitAfterHang = new WaitAction(500);
//        waitAfterHang.setName("waitAfterHang");
//        waitAfterHang.setDependentActions(lowerSlidesHalf1);
//        redAutoBasket.addAction(waitAfterHang);
//
//        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
//                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
//        openClaw.setName("openClaw");
//        openClaw.setDependentActions(waitAfterHang);
//        redAutoBasket.addAction(openClaw);
//
//        OuttakeDownReady outtakeDownReady1 = new OuttakeDownReady(outtake);
//        outtakeDownReady1.setName("outtakeDownReady1");
//        outtakeDownReady1.setDependentActions(openClaw);
//        redAutoBasket.addAction(outtakeDownReady1);
        //===============end of first specimen===============



        //================begin of first basket====================
        PurePursuitAction pushSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        pushSamples.setName("pushSamples");
        pushSamples.setDependentActions(waitAtStart); // waitAfterHang
        pushSamples.addPoint( -620, 475, -90); //turning point
        pushSamples.addPoint(-1330, 500, -180); //
        pushSamples.addPoint(-1330, 800, -180);// before push
        pushSamples.addPoint(basketOuttakeXPos, basketOuttakeYPos, -135);

        //==============SECONDSAMPLE================

        pushSamples.addPoint(-1330, 775, -180);//
        pushSamples.addPoint(-1330, 1050, -180); //before push
        pushSamples.addPoint(basketOuttakeXPos, basketOuttakeYPos, -135);

        //=============THIRDSAMPLE===================

        pushSamples.addPoint(-1330, 1050, -180);//
        pushSamples.addPoint(-1330, 1175, -180); //before push
        pushSamples.addPoint(basketOuttakeXPos, basketOuttakeYPos, -135);

        redAutoBasket.addAction(pushSamples);

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            maintainLS.update();

            redAutoBasket.updateCheckDone();

            initAuto.update();

        }

    }
}