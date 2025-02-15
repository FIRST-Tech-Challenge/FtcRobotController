package com.kalipsorobotics.intoTheDeep;

import android.os.Process;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.intake.IntakeCounterWeight;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
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
import com.qualcomm.hardware.ams.AMSColorSensor;
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
        SharedData.resetOdometryPosition();

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
//
//        KServoAutoAction ratchetUnlock = new KServoAutoAction(intakeClaw.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
//        ratchetUnlock.setName("ratchetUnlock");
//        ratchetUnlock.setDependentActions(wallToBarHangAction);
//        redAutoSpecimen.addAction(ratchetUnlock);

        WaitAction waitForLinkageExtend = new WaitAction(3000);
        waitForLinkageExtend.setName("waitForLinkageExtend");
        waitForLinkageExtend.setDependentActions(wallToBarHangAction);
        redAutoSpecimen.addAction(waitForLinkageExtend);
//
//        KServoAutoAction extendLinkage = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(),
//                IntakeClaw.INTAKE_LINKAGE_EXTEND_POS);
//        extendLinkage.setName("extendLinkage");
//        extendLinkage.setDependentActions(waitForLinkageExtend);
//        redAutoSpecimen.addAction(extendLinkage);


        IntakeCounterWeight intakeCounterWeight = new IntakeCounterWeight(intakeClaw, outtake);
        intakeCounterWeight.setName("intakeCounterWeight");
        intakeCounterWeight.setDependentActions(waitForLinkageExtend);
        redAutoSpecimen.addAction(intakeCounterWeight);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setMaxTimeOutMS(12000);
        moveFloorSamples.setTelemetry(telemetry);
        moveFloorSamples.setDependentActions(wallToBarHangAction);

        //move to sample
        moveFloorSamples.addPoint(-570, 230, -135, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);
        moveFloorSamples.addPoint(-570, -425, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST);
        moveFloorSamples.addPoint( -670, -425, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_FAST); //y
        moveFloorSamples.addPoint(-1330, -500, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //y -475

        //first sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);//before push y=800
        //moveFloorSamples.addPoint(-580, -775, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-510, -875, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-280, -675, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW); //y -800
        moveFloorSamples.addPoint(-1330, -1000, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);//// beforepush
        //moveFloorSamples.addPoint(-580, -1000, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-510,-1100,-180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-280,-900,-180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);
        //-367.5
        //moveFloorSamples.addPoint(-175, -1065, -180);?

        // 3rd sample push depot
        final double THRID_SAMPLE_PUSHING_Y = -1190;
//        moveFloorSamples.addPoint(-1300, -1050, -180);
        moveFloorSamples.addPoint(-1300, -950, -180,  PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-1300, THRID_SAMPLE_PUSHING_Y, -180, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE_SLOW);//before push //-1300, -1175 // -1175 BEFORE DARREN CHANGED
        moveFloorSamples.addPoint(-600, THRID_SAMPLE_PUSHING_Y, -180, PurePursuitAction.P_XY,
                PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, THRID_SAMPLE_PUSHING_Y, -180,
                PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW); // -1065 // WAllPICKUP = -150
//        moveFloorSamples.addPoint(-350, -1065, -180);//move back out to avoid sample carry //y = -1200 y = 375
        redAutoSpecimen.addAction(moveFloorSamples);
//
        WaitAction waitBeforeSpecimenReady = new WaitAction(9000); // I swears its ok waiting for transfer ready
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

        IntakeTransferReady intakeTransferReady = new IntakeTransferReady(intakeClaw);
        intakeTransferReady.setName("intakeTransferReady");
        intakeTransferReady.setDependentActions(moveFloorSamples);
        redAutoSpecimen.addAction(intakeTransferReady);
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
                outtake,350); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setTelemetry(telemetry);
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of third specimen===========

        //===============start of fourth specimen==============
        WallToBarHangRoundTrip wallToBarHangRoundTrip4 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,410);
        wallToBarHangRoundTrip4.setName("wallToBarHangRoundTrip4");
        wallToBarHangRoundTrip4.setTelemetry(telemetry);
        wallToBarHangRoundTrip4.setDependentActions(wallToBarHangRoundTrip3);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip4);
        //================end of specimen 4================

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        redAutoSpecimen.printWithDependentActions();

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);

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

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}
