package com.kalipsorobotics.test.outtake;

import com.kalipsorobotics.actions.outtake.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.OuttakePigeonAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeSlideAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OuttakeTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = new Outtake(opModeUtilities);
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake);
        OuttakeClawAction outtakeClawAction = new OuttakeClawAction(outtake);
        OuttakePigeonAction outtakePigeonAction = new OuttakePigeonAction(outtake);

        double pivotPosition = 0.5;
        double clawPosition = 0.5;
        double pigeonPosition = 0.5;

        outtakePivotAction.setPosition(pivotPosition);
        outtakeClawAction.setPosition(clawPosition);
        outtakePigeonAction.setPosition(pigeonPosition);

        waitForStart();
        while (opModeIsActive()) {


            if (-gamepad1.right_stick_y > 0.1) {
                outtakeSlideAction.setPower(-gamepad1.right_stick_y);
            } else if (-gamepad1.right_stick_y < -0.1) {
                outtakeSlideAction.setPower(-gamepad1.right_stick_y);
            } else {
                outtakeSlideAction.idle();
            }

            if (gamepad1.y) {
                pivotPosition = pivotPosition + 0.001;
                outtakePivotAction.setPosition(pivotPosition);
            } else if (gamepad1.x) {
                pivotPosition = pivotPosition - 0.001;
                outtakePivotAction.setPosition(pivotPosition);
            }

            if (gamepad1.b) {
                clawPosition = clawPosition + 0.01;
                outtakeClawAction.setPosition(clawPosition);
            } else if (gamepad1.a) {
                clawPosition = clawPosition - 0.01;
                outtakeClawAction.setPosition(clawPosition);
            }

            if (gamepad1.dpad_left) {
                pigeonPosition = pigeonPosition + 0.001;
                outtakePigeonAction.setPosition(pigeonPosition);
            } else if (gamepad1.dpad_right) {
                pigeonPosition = pigeonPosition - 0.001;
                outtakePigeonAction.setPosition(pigeonPosition);
            }

            telemetry.addData("pigeon position", pigeonPosition);
            telemetry.addData("claw position", clawPosition);
            telemetry.addData("pivot position", pivotPosition);
            telemetry.update();
        }
    }
}

