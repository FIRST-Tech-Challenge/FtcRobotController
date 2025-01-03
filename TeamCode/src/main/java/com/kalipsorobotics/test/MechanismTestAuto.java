package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class MechanismTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        Outtake outtake = Outtake.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        MoveLSAction maintenanceLS = new MoveLSAction(outtake, 0
        );

        KServoAutoAction outtakePivotActionIn = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);

        KActionSet redAutoSpecimen = new KActionSet();

        Intake intake = new Intake(opModeUtilities);

        MoveLSAction moveOuttakeLSAction = new MoveLSAction(outtake, 480);
        moveOuttakeLSAction.setName("moveOuttakeLSAction");

        WaitAction waitAction = new WaitAction(3000);
        waitAction.setDependentActions(moveOuttakeLSAction);
        waitAction.setName("waitAction");

        MoveLSAction moveOuttakeLSAction2 = new MoveLSAction(outtake, 200);
        moveOuttakeLSAction2.setDependentActions(waitAction);
        moveOuttakeLSAction2.setName("moveOuttakeLSAction2");

        WaitAction waitAction2 = new WaitAction(3000);
        waitAction2.setDependentActions(moveOuttakeLSAction2);
        waitAction2.setName("waitAction2");

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);

//        IntakeReadyAction intakeReadyAction = new IntakeReadyAction();
        //BasketReadyAction basketReadyAction = new BasketReadyAction();

        while (opModeInInit()) {
            if (gamepad1.a) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(outtakePivotActionIn);
                telemetry.addLine("done init hang specimen ready");
                telemetry.update();
            }
            if (gamepad1.b) {
                redAutoSpecimen.clear();
                telemetry.addLine("done auto init");
                telemetry.update();
            }
            if (gamepad1.x) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(moveOuttakeLSAction);
                redAutoSpecimen.addAction(waitAction);
                redAutoSpecimen.addAction(moveOuttakeLSAction2);
                redAutoSpecimen.addAction(waitAction2);
                telemetry.addLine("done Move Ls");
                telemetry.update();
            }
            if (gamepad1.y) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(specimenWallReady);
                telemetry.addLine("done wall ready");
                telemetry.update();
            }
        }

        redAutoSpecimen.printWithDependentActions();
        waitForStart();
        while (opModeIsActive()) {
            maintenanceLS.update();
            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();

        }

    }
}
