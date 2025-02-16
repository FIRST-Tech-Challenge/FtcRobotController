package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.SampleSweepingReady;
import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Disabled

public class AutoSpecimenChopping extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet autoSpecimenChopping = new KActionSet();
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
            setAutoDelayAction.updateCheckDone();
        }

        WaitAction delayBeforeStart = new WaitAction(setAutoDelayAction.getTimeMs());
        delayBeforeStart.setName("delayBeforeStart");
        autoSpecimenChopping.addAction(delayBeforeStart);

        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, 230);
        wallToBarHangAction.setName("wallToBarHangAction");
        wallToBarHangAction.setTelemetry(telemetry);
        wallToBarHangAction.setDependentActions(delayBeforeStart);
        autoSpecimenChopping.addAction(wallToBarHangAction);

        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(wallToBarHangAction);
        autoSpecimenChopping.addAction(outtakeTransferReady);
//
//        WaitAction waitBeforeSweepReady = new WaitAction();

        PurePursuitAction moveToSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSample1.setName("moveToSample1");
        moveToSample1.setMaxTimeOutMS(12000);
        moveToSample1.setTelemetry(telemetry);
        moveToSample1.setDependentActions(wallToBarHangAction);
        moveToSample1.addPoint(-500, -300, -145, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE_SLOW);
        autoSpecimenChopping.addAction(moveToSample1);

        SampleIntakeReady sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw);
        sampleIntakeReady.setDependentActions(wallToBarHangAction);
        autoSpecimenChopping.addAction(sampleIntakeReady);

        SampleIntakeAction sampleIntakeAction = new SampleIntakeAction(intakeClaw);
        sampleIntakeAction.setDependentActions(sampleIntakeReady, moveToSample1);
        autoSpecimenChopping.addAction(sampleIntakeAction);

        PurePursuitAction turnToDepot = new PurePursuitAction(driveTrain, wheelOdometry);
        turnToDepot.setDependentActions(sampleIntakeAction, moveToSample1);
        turnToDepot.addPoint(-500, -300, -45, PurePursuitAction.P_XY, PurePursuitAction.P_ANGLE_FAST);
        autoSpecimenChopping.addAction(turnToDepot);

        ExecutorService executorService = Executors.newSingleThreadExecutor();

        autoSpecimenChopping.printWithDependentActions();

        waitForStart();

        OpModeUtilities.runOdometryExecutorService(executorService, wheelOdometry);


        while (opModeIsActive()) {
            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();
            autoSpecimenChopping.updateCheckDone();
        }

        OpModeUtilities.shutdownExecutorService(executorService);

    }
}
