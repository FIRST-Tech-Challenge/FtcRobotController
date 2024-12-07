package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.intake.IntakeReadyAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
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
        MoveOuttakeLSAction maintenanceLS = new MoveOuttakeLSAction(outtake, 0
        );

        KServoAutoAction outtakePivotActionIn = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_OUT_POS);

        KActionSet redAutoSpecimen = new KActionSet();

        Intake intake = new Intake(opModeUtilities);

        InitAuto initAuto = new InitAuto(intake, outtake);

        MoveOuttakeLSAction moveOuttakeLSAction = new MoveOuttakeLSAction(outtake, 400);

        WaitAction waitAction = new WaitAction(3000);
        waitAction.setDependantActions(moveOuttakeLSAction);

        MoveOuttakeLSAction moveOuttakeLSAction2 = new MoveOuttakeLSAction(outtake, 200);
        moveOuttakeLSAction2.setDependantActions(waitAction);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);

        IntakeReadyAction intakeReadyAction = new IntakeReadyAction();
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
                redAutoSpecimen.addAction(initAuto);
                telemetry.addLine("done auto init");
                telemetry.update();
            }
            if (gamepad1.x) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(moveOuttakeLSAction);
                redAutoSpecimen.addAction(waitAction);
                redAutoSpecimen.addAction(moveOuttakeLSAction2);
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

        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {
            maintenanceLS.update();
            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();

        }

    }
}
