package com.kalipsorobotics.test.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.OpModeUtilities;
@TeleOp
public class IntakeTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Intake intake = new Intake(opModeUtilities);
        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake);
        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);

        waitForStart();

        double position = 0.2;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intakeNoodleAction.run();
            } else if (gamepad1.b) {
                intakeNoodleAction.stop();
            }

            if (gamepad1.y) {
                position = position + 0.01;
                intakePivotAction.goToPosition(position);
            } else if (gamepad1.x) {
                position = position - 0.01;
                intakePivotAction.goToPosition(position);
            }

        }

    }


}
