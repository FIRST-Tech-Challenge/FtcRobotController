package com.kalipsorobotics.test.intake;

import com.kalipsorobotics.actions.intake.IntakeDoorAction;
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
        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake, 0, false);
        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);
        IntakeDoorAction intakeDoorAction = new IntakeDoorAction(intake);


        double pivotPosition = 0.5;
        double doorPosition = 0.5;
        double linkagePosition = 0.5;
        double linkageSpeed =0.005;

        intakePivotAction.moveDown();
        intakeDoorAction.close();


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.left_trigger > 0.5) {
                intakeNoodleAction.run();
            } else {
                intakeNoodleAction.stop();
            }

            if (gamepad1.y) {
                pivotPosition = pivotPosition + 0.01;
                intakePivotAction.setPosition(pivotPosition);
            } else if (gamepad1.x) {
                pivotPosition = pivotPosition - 0.01;
                intakePivotAction.setPosition(pivotPosition);
            }

            if (gamepad1.b) {
                doorPosition = doorPosition + 0.01;
                intakeDoorAction.open();
            } else if (gamepad1.a) {
                doorPosition = doorPosition - 0.01;
                intakeDoorAction.close();
            }

            if (gamepad1.dpad_up) {
                linkagePosition = linkagePosition + linkageSpeed;
            } else if (gamepad1.dpad_down) {
                linkagePosition = linkagePosition - linkageSpeed;
            }

            telemetry.addData("doorPosition", doorPosition);
            telemetry.addData("linkagePosition", linkagePosition);
            telemetry.addData("pivot position", pivotPosition);
            telemetry.update();

        }

    }


}
