package com.kalipsorobotics.intoTheDeep;

import android.util.Log;

import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.kalipsorobotics.modules.RevDistance;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        Rev2mDistanceSensor revDistance = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance");
        Rev2mDistanceSensor revDistance2 = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance2");

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");
        initAuto.update();

        telemetry.addLine("init finished");
        telemetry.update();

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMs());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoSpecimen.addAction(delayBeforeStart);

        //================begin of first specimen====================
        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, revDistance2,240);
        wallToBarHangAction.setName("wallToBarHangAction");
        wallToBarHangAction.setDependentActions(delayBeforeStart);
        redAutoSpecimen.addAction(wallToBarHangAction);
        //===============end of first specimen===============

//        WaitAction waitForHangFinish = new WaitAction(100);
//        waitForHangFinish.setDependentActions(wallToBarHangAction);
//        redAutoSpecimen.addAction(waitForHangFinish);

        //================beginning of push================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(outtakeTransferReady);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry,1.0/100.0);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependentActions(wallToBarHangAction);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90); //y -500
        moveFloorSamples.addPoint(-1330, -500, -180); //y -475
        moveFloorSamples.addPoint(-1330, -800, -180);// before push
        moveFloorSamples.addPoint(-380, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180); //y -800
        moveFloorSamples.addPoint(-1330, -1065, -180);// before push
        moveFloorSamples.addPoint(-380,-1065,-180); //-367.5
        //moveFloorSamples.addPoint(-175, -1065, -180);

        // 3rd sample push depot
//        moveFloorSamples.addPoint(-1300, -1050, -180);
        moveFloorSamples.addPoint(-1300, -1210, -180);//before push //-1300, -1175 // -1175 BEFORE DARREN CHANGED
        moveFloorSamples.addPoint(-240, -1210, -180);
        moveFloorSamples.addPoint(-350, -1065, -180);//move back out to avoid sample carry //y = -1200 y = 375
        redAutoSpecimen.addAction(moveFloorSamples);

        WaitAction waitBeforeSpecimenReady = new WaitAction(7000); // I swears its ok waiting for transfer ready
        waitBeforeSpecimenReady.setName("waitBeforeSpecimenReady");
        waitBeforeSpecimenReady.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(waitBeforeSpecimenReady);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(waitBeforeSpecimenReady);
        redAutoSpecimen.addAction(specimenWallReady);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, 1.0/800.0); //2200
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(moveFloorSamples, specimenWallReady);
        //to depot for specimen
        //moveToDepot.addPoint(-380, -1050, -180); //-380, -615
        moveToDepot.setMaxTimeOutMS(1200);
        moveToDepot.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, -1065, -180);//-130, -615
        redAutoSpecimen.addAction(moveToDepot);
        //==============end of pushing================

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction = new WallPickupDistanceSensorAction(outtake, revDistance, moveToDepot);
        wallPickupDistanceSensorAction.setDependentActions(specimenWallReady);
        redAutoSpecimen.addAction(wallPickupDistanceSensorAction);

        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, revDistance, revDistance2, 290); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setDependentActions(wallPickupDistanceSensorAction, moveToDepot);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, revDistance,revDistance2,390); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of third specimen===========

        //===============start of fourth specimen==============
        WallToBarHangRoundTrip wallToBarHangRoundTrip4 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, revDistance,revDistance2,525);
        wallToBarHangRoundTrip4.setName("wallToBarHangRoundTrip4");
        wallToBarHangRoundTrip4.setDependentActions(wallToBarHangRoundTrip3);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip4);
        //================end of specimen 4================


        redAutoSpecimen.printWithDependentActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();


//            Log.d("WallDistance:", String.valueOf(revDistance.getDistance(DistanceUnit.MM)));
//            Log.d("BarDistance:", String.valueOf(revDistance2.getDistance(DistanceUnit.MM)));
            redAutoSpecimen.updateCheckDone();


        }

    }
}
