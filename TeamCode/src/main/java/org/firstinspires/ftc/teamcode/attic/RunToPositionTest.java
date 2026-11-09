package org.firstinspires.ftc.teamcode.attic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;

@Disabled
@TeleOp(name = "Run To Position Test", group = "Test")
public class RunToPositionTest extends OrangeBotBaseOpMode {

    private double INTAKE_POWER;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        waitForStart();

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(INTAKE_POWER);

        while (opModeIsActive()) {

            if (gamepad1.bWasPressed()) {
                intake.setTargetPosition(350);
            }
            if (gamepad1.aWasPressed()) {
                intake.setTargetPosition(700);
            }
            if (gamepad1.xWasPressed()) {
                intake.setTargetPosition(1050);
            }
            if (gamepad1.yWasPressed()) {
                intake.setTargetPosition(1400);
            }

            telemetry.addData("Current position", intake.getCurrentPosition());
            telemetry.update();
        }

    }


}