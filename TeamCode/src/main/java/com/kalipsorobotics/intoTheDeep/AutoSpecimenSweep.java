package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.SetAutoDelayAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.WallToBarHangAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoSpecimenSweep extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet autoSpecimenSweep = new KActionSet();
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
        autoSpecimenSweep.addAction(delayBeforeStart);




        WallToBarHangAction wallToBarHangAction = new WallToBarHangAction(driveTrain, wheelOdometry, outtake, 230);
        wallToBarHangAction.setName("wallToBarHangAction");
        wallToBarHangAction.setTelemetry(telemetry);
        wallToBarHangAction.setDependentActions(delayBeforeStart);
        autoSpecimenSweep.addAction(wallToBarHangAction);
    }
}
