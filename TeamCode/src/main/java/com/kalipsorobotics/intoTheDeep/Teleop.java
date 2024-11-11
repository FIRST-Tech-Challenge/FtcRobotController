package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Intake intake = new Intake(opModeUtilities);
        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake);
        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);
        IntakeDoorAction intakeDoorAction = new IntakeDoorAction(intake);
        IntakeLinkageAction intakeLinkageAction = new IntakeLinkageAction(intake);

        boolean prevGamePadX = false;
        boolean prevGamePadB = false;
        boolean prevGamePadA = false;

        waitForStart();
        while (opModeIsActive()) {

            //Noodles
            if (gamepad2.left_trigger > 0.5) {
                intakeNoodleAction.run();
            } else if (gamepad2.left_bumper) {
                intakeNoodleAction.reverse();
            } else {
                intakeNoodleAction.stop();
            }

            //Pivot TODO make generic toggle
            if (gamepad2.x && !prevGamePadX) {
                intakePivotAction.togglePosition();
            }
            prevGamePadX = gamepad2.x;

            //Door
            if (gamepad2.b && !prevGamePadB) {
                intakeDoorAction.togglePosition();
            }
            prevGamePadB = gamepad2.b;

            //Linkage
            if (gamepad2.a && !prevGamePadA) {
                intakeLinkageAction.togglePosition();
            }
            prevGamePadA = gamepad2.a;



        }
    }
}
