package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.HangSpecimenReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class MechanismTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);


        KServoAutoAction outtakePivotActionIn = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_OUT_POS);

        KActionSet redAutoSpecimen = new KActionSet();
        while (opModeInInit()) {
            if (gamepad1.a) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(outtakePivotActionIn);
                telemetry.addLine("done init hang specimen ready");
                telemetry.update();
            }
            if (gamepad1.b) {

            }
        }

        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();

        }

    }
}
