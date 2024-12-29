package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedAutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoSpecimen = new KActionSet();
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        IntakeClaw intakeClaw = new IntakeClaw(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        // Target can always be 0 because Hung said so
        MoveOuttakeLSAction maintainLS = new MoveOuttakeLSAction(outtake, 0);
//                MoveLSAction.globalLinearSlideMaintainTicks);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(300);
        waitAtStart.setName("waitAtStart");
        redAutoSpecimen.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.setMaxTimeOutMS(4000);
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, 300, 0);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        redAutoSpecimen.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        redAutoSpecimen.addAction(specimenHangReady1);

        SpecimenHang specimenHang1 = new SpecimenHang(outtake);
        specimenHang1.setName("specimenHang1");
        specimenHang1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
        redAutoSpecimen.addAction(specimenHang1);
        //===============end of first specimen===============



        //================beginning of push================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(specimenHang1);
        redAutoSpecimen.addAction(outtakeTransferReady);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependentActions(specimenHang1);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90); //y -500
        moveFloorSamples.addPoint(-1330, -500, -180); //y -475
        moveFloorSamples.addPoint(-1330, -800, -180);// before push
        moveFloorSamples.addPoint(-240, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180); //y -800
        moveFloorSamples.addPoint(-1330, -1055, -180);// before push
        moveFloorSamples.addPoint(-375, -1055, -180);

        //third sample to depot
//        moveFloorSamples.addPoint(-1300, -1050, -180);
//        moveFloorSamples.addPoint(-1300, -1175, -180);//before push //-1300, -1175
//        moveFloorSamples.addPoint(-240, -1175, -180);
//        moveFloorSamples.addPoint(-430, -1175, -180);//move back out to avoid sample carry
        redAutoSpecimen.addAction(moveFloorSamples);

//        WaitAction waitBeforeSpecimenReady = new WaitAction(1000);
//        waitBeforeSpecimenReady.setName("waitBeforeSpecimenReady");
//        waitBeforeSpecimenReady.setDependentActions(specimenHang1);
//        redAutoSpecimen.addAction(waitBeforeSpecimenReady);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(moveFloorSamples);
        redAutoSpecimen.addAction(specimenWallReady);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, 1.0/2000.0);
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveFloorSamples, specimenWallReady);
        //to depot for specimen
        //moveToDepot.addPoint(-380, -1050, -180); //-380, -615
        moveToDepot.addPoint(-135, -1055, -180); //-130, -615
        redAutoSpecimen.addAction(moveToDepot);
        //==============end of pushing================


        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 375); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setDependentActions(moveToDepot);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 450); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of third specimen===========

        //===============start of fourth specimen==============
//        WallToBarHangRoundTrip wallToBarHangRoundTrip4 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
//                outtake, 525);
//        wallToBarHangRoundTrip4.setName("wallToBarHangRoundTrip4");
//        wallToBarHangRoundTrip4.setDependentActions(wallToBarHangRoundTrip3);
//        redAutoSpecimen.addAction(wallToBarHangRoundTrip4);
        //================end of specimen 4================



        initAuto.update();

        redAutoSpecimen.printWithDependentActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            maintainLS.update();

            redAutoSpecimen.updateCheckDone();

            initAuto.update();

        }

    }
}
