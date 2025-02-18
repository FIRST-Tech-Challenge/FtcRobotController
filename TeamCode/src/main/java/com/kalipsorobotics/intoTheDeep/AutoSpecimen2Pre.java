package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.autoActions.FirstWallToBarRoundTrip;
import com.kalipsorobotics.actions.autoActions.WallToBarMoveHang;
import com.kalipsorobotics.actions.intake.IntakeCounterWeight;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.autoActions.FirstWallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(name = "AutoSpecimen  2 SPECIMEN + 2 PUSH ")
public class AutoSpecimen2Pre extends LinearOpMode {


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

        //================begin of first specimen=============//
        FirstWallToBarRoundTrip firstWallToBarRoundTrip = new FirstWallToBarRoundTrip(driveTrain, wheelOdometry,
                outtake, 200);
        firstWallToBarRoundTrip.setName("firstWallToBarRoundTrip");
        firstWallToBarRoundTrip.setTelemetry(telemetry);
        firstWallToBarRoundTrip.setDependentActions(delayBeforeStart);
        redAutoSpecimen.addAction(firstWallToBarRoundTrip);
        //===============end of first specimen===============//

        //================begin of second specimen=============//
        WallToBarMoveHang wallToBarMoveHang = new WallToBarMoveHang(driveTrain, wheelOdometry, outtake, 250);
        wallToBarMoveHang.setName("wallToBarHangRoundTripPre2");
        wallToBarMoveHang.setTelemetry(telemetry);
        wallToBarMoveHang.setDependentActions(firstWallToBarRoundTrip);
        redAutoSpecimen.addAction(wallToBarMoveHang);
        //===============end of second specimen===============//


        //================beginning of push================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(wallToBarMoveHang);
        redAutoSpecimen.addAction(outtakeTransferReady);

        WaitAction waitForLinkageExtend = new WaitAction(3000);
        waitForLinkageExtend.setName("waitForLinkageExtend");
        waitForLinkageExtend.setDependentActions(wallToBarMoveHang);
        redAutoSpecimen.addAction(waitForLinkageExtend);

        IntakeCounterWeight intakeCounterWeight = new IntakeCounterWeight(intakeClaw, outtake);
        intakeCounterWeight.setName("intakeCounterWeight");
        intakeCounterWeight.setDependentActions(waitForLinkageExtend);
        redAutoSpecimen.addAction(intakeCounterWeight);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setMaxTimeOutMS(12000);
        moveFloorSamples.setTelemetry(telemetry);
        moveFloorSamples.setDependentActions(wallToBarMoveHang);

        //move to sample:
        moveFloorSamples.addPoint(-570, 130, -90, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); //
        //turning:
        moveFloorSamples.addPoint(-900, -500, -175, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_FAST); // y
        //move back:
        moveFloorSamples.addPoint(-1330, -500, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE);
        //y -475


        //first sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);//before push y=800
        moveFloorSamples.addPoint(-610, -825, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(-380, -675, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE_SLOW);

        final double WALL_PICKUP_Y = -1108;
        //second sample to depot
        moveFloorSamples.addPoint(-1000, -675, -180, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE); //y
        moveFloorSamples.addPoint(-1400, -675, -180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE); //y
        moveFloorSamples.addPoint(-1400, WALL_PICKUP_Y, -180, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);//
        moveFloorSamples.addPoint(-600,WALL_PICKUP_Y,-180, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_SLOW);
        moveFloorSamples.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X + 25, WALL_PICKUP_Y, -180,
                    PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);
        redAutoSpecimen.addAction(moveFloorSamples);

        WaitAction waitBeforeSpecimenReady = new WaitAction(6000); // I swears its ok waiting
        waitBeforeSpecimenReady.setName("waitBeforeSpecimenReady");
        waitBeforeSpecimenReady.setTelemetry(telemetry);
        waitBeforeSpecimenReady.setDependentActions(wallToBarMoveHang);
        redAutoSpecimen.addAction(waitBeforeSpecimenReady);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setTelemetry(telemetry);
        specimenWallReady.setDependentActions(waitBeforeSpecimenReady);
        redAutoSpecimen.addAction(specimenWallReady);

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



        //=============begin of third specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 320); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setTelemetry(telemetry);
        wallToBarHangRoundTrip2.setDependentActions(wallPickupDistanceSensorAction, moveFloorSamples); //move to distance
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of third specimen==============

        //============begin of fourth================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,380); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setTelemetry(telemetry);
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of fourth specimen===========


        //===============do not touch -- essential for auto 0_0======================/
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
//            telemetry.addData("X", SharedData.getOdometryPosition().getX());
//            telemetry.addData("Y", SharedData.getOdometryPosition().getY());
//            telemetry.addData("Theta", SharedData.getOdometryPosition().getTheta());
//            telemetry.addData("ls", MoveLSAction.getGlobalLinearSlideMaintainTicks());
//
//            telemetry.addData("moveFloorSamples",moveFloorSamples.toString());
//            //telemetry.addData("detectWallPickup",wallPickupDistanceSensorAction.toString());
//            telemetry.addData("wallToBarRound1",wallToBarHangRoundTrip2.toString());
//
//            telemetry.update();

        }

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}
