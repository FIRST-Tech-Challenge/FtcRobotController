package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoSpecimen = new KActionSet();
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

        //================begin of first specimen====================
        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, 300);
        wallToBarHangAction.setName("wallToBarHangAction");
        redAutoSpecimen.addAction(wallToBarHangAction);
        //===============end of first specimen===============



        //================beginning of push================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(outtakeTransferReady);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry,1.0/300.0);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependentActions(wallToBarHangAction);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90); //y -500
        moveFloorSamples.addPoint(-1330, -500, -180); //y -475
        moveFloorSamples.addPoint(-1330, -800, -180);// before push
        moveFloorSamples.addPoint(-240, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180); //y -800
        moveFloorSamples.addPoint(-1330, -1065, -180);// before push
        moveFloorSamples.addPoint(-367.5,-1065,-180);
        //moveFloorSamples.addPoint(-175, -1065, -180);

        //third sample to depot
//        moveFloorSamples.addPoint(-1300, -1050, -180);
//        moveFloorSamples.addPoint(-1300, -1175, -180);//before push //-1300, -1175
//        moveFloorSamples.addPoint(-240, -1175, -180);
//        moveFloorSamples.addPoint(-430, -1175, -180);//move back out to avoid sample carry
        redAutoSpecimen.addAction(moveFloorSamples);

        WaitAction waitBeforeSpecimenReady = new WaitAction(7000); // I swears its ok waiting for transfer ready
        waitBeforeSpecimenReady.setName("waitBeforeSpecimenReady");
        waitBeforeSpecimenReady.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(waitBeforeSpecimenReady);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(waitBeforeSpecimenReady);
        redAutoSpecimen.addAction(specimenWallReady);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, WallToBarHangRoundTrip.WALL_PICKUP_PID_VALUE); //2200
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveFloorSamples, specimenWallReady);
        //to depot for specimen
        //moveToDepot.addPoint(-380, -1050, -180); //-380, -615
        moveToDepot.setMaxTimeOutMS(3000);
        moveToDepot.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, -1065, -180);//-130, -615
        redAutoSpecimen.addAction(moveToDepot);
        //==============end of pushing================


        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 400); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setDependentActions(moveFloorSamples, specimenWallReady, moveToDepot);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 500); //500 //450
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

            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();

            redAutoSpecimen.updateCheckDone();

        }

    }
}
