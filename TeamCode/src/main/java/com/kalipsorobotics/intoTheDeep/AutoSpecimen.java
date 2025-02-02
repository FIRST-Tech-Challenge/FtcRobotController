package com.kalipsorobotics.intoTheDeep;

import android.os.Process;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous
public class AutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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

        Rev2mDistanceSensor revDistanceClaw = hardwareMap.get(Rev2mDistanceSensor.class, "revDistanceClaw");
        Rev2mDistanceSensor revDistanceBottom = hardwareMap.get(Rev2mDistanceSensor.class, "revDistanceBottom");

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");
//        initAuto.update();

        telemetry.addLine("init finished");

        SetAutoDelayAction setAutoDelayAction = new SetAutoDelayAction(opModeUtilities, gamepad1);
        setAutoDelayAction.setName("setAutoDelayAction");
        setAutoDelayAction.setTelemetry(telemetry);

        while(!setAutoDelayAction.getIsDone() && opModeInInit()) {
            long timestamp = System.currentTimeMillis();
            telemetry.addData("X", SharedData.getOdometryPosition().getX());
            telemetry.addData("Y",SharedData.getOdometryPosition().getY());
            telemetry.addData("Theta",SharedData.getOdometryPosition().getTheta());
            telemetry.addData("LinearSlide", MoveLSAction.getGlobalLinearSlideMaintainTicks());
            telemetry.update();
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMs());
        delayBeforeStart.setName("delayBeforeStart");
        redAutoSpecimen.addAction(delayBeforeStart);

        //================begin of first specimen====================
        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, 230);
        wallToBarHangAction.setName("wallToBarHangAction");
        wallToBarHangAction.setTelemetry(telemetry);
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

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setMaxTimeOutMS(12000);
        moveFloorSamples.setTelemetry(telemetry);
        moveFloorSamples.setDependentActions(wallToBarHangAction);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //y -500
        moveFloorSamples.addPoint(-1330, -500, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //y -475
        moveFloorSamples.addPoint(-1330, -800, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);// before push
        moveFloorSamples.addPoint(-380, -800, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //y -800
        moveFloorSamples.addPoint(-1330, -1065, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);// before push
        moveFloorSamples.addPoint(-380,-1065,-180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //-367.5
        //moveFloorSamples.addPoint(-175, -1065, -180);

        // 3rd sample push depot
        double THRID_SAMPLE_PUSHING_Y = -1155;
//        moveFloorSamples.addPoint(-1300, -1050, -180);
        moveFloorSamples.addPoint(-1300, THRID_SAMPLE_PUSHING_Y, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE);//before push //-1300, -1175 // -1175 BEFORE DARREN CHANGED
        moveFloorSamples.addPoint(-600, THRID_SAMPLE_PUSHING_Y, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE);
        moveFloorSamples.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, THRID_SAMPLE_PUSHING_Y, -180, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE); // -1065 // WAllPICKUP = -150
//        moveFloorSamples.addPoint(-350, -1065, -180);//move back out to avoid sample carry //y = -1200 y = 375
        redAutoSpecimen.addAction(moveFloorSamples);
//
        WaitAction waitBeforeSpecimenReady = new WaitAction(7000); // I swears its ok waiting for transfer ready
        waitBeforeSpecimenReady.setName("waitBeforeSpecimenReady");
        waitBeforeSpecimenReady.setTelemetry(telemetry);
        waitBeforeSpecimenReady.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(waitBeforeSpecimenReady);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setTelemetry(telemetry);
        specimenWallReady.setDependentActions(waitBeforeSpecimenReady);
        redAutoSpecimen.addAction(specimenWallReady);

//        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain, wheelOdometry); //2200
//        moveToDepot.setName("moveToDepot");
//        moveToDepot.setTelemetry(telemetry);
//        moveToDepot.setDependentActions(moveFloorSamples, specimenWallReady);
//        //to depot for specimen
//        //moveToDepot.addPoint(-380, -1050, -180); //-380, -615
//        moveToDepot.setMaxTimeOutMS(1200);
//        moveToDepot.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X+25, -1065, -180);// y = -1065 x = wall_pickupX
//        redAutoSpecimen.addAction(moveToDepot);
        //==============end of pushing================//

        WallPickupDistanceSensorAction wallPickupDistanceSensorAction = new WallPickupDistanceSensorAction(outtake, moveFloorSamples, driveTrain);
        wallPickupDistanceSensorAction.setName("wallPickupDistanceSensor");
        wallPickupDistanceSensorAction.setTelemetry(telemetry);
        wallPickupDistanceSensorAction.setDependentActions(specimenWallReady);
        redAutoSpecimen.addAction(wallPickupDistanceSensorAction);

        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 290); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setTelemetry(telemetry);
        wallToBarHangRoundTrip2.setDependentActions(wallPickupDistanceSensorAction);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,390); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setTelemetry(telemetry);
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of third specimen===========

        //===============start of fourth specimen==============
        WallToBarHangRoundTrip wallToBarHangRoundTrip4 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,525);
        wallToBarHangRoundTrip4.setName("wallToBarHangRoundTrip4");
        wallToBarHangRoundTrip4.setTelemetry(telemetry);
        wallToBarHangRoundTrip4.setDependentActions(wallToBarHangRoundTrip3);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip4);
        //================end of specimen 4================

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        redAutoSpecimen.printWithDependentActions();

        waitForStart();

        executorService.submit(() -> {
            android.os.Process.setThreadPriority(Process.THREAD_PRIORITY_FOREGROUND);
            while (true) {
                wheelOdometry.updatePosition();
            }

        });

        while (opModeIsActive()) {


            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();


//            Log.d("WallDistance:", String.valueOf(revDistanceClaw.getDistance(DistanceUnit.MM)));
//            Log.d("BarDistance:", String.valueOf(revDistanceBottom.getDistance(DistanceUnit.MM)));
            redAutoSpecimen.updateCheckDone();

//            telemetry.addData("claw", revDistanceClaw.getDistance(DistanceUnit.MM));
//            telemetry.addData("bottom", revDistanceBottom.getDistance(DistanceUnit.MM));
            telemetry.addData("X", SharedData.getOdometryPosition().getX());
            telemetry.addData("Y", SharedData.getOdometryPosition().getY());
            telemetry.addData("Theta", SharedData.getOdometryPosition().getTheta());
            telemetry.addData("ls", MoveLSAction.getGlobalLinearSlideMaintainTicks());

            telemetry.addData("moveFloorSamples",moveFloorSamples.toString());
            telemetry.addData("detectWallPickup",wallPickupDistanceSensorAction.toString());
            telemetry.addData("wallToBarRound1",wallToBarHangRoundTrip2.toString());

            telemetry.update();

        }
        executorService.shutdown();

    }
}
