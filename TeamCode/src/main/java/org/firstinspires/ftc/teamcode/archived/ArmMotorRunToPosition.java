package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ArmMotorRunToPosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        int armPosition = 0;

        // Reset the encoder during initialization
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            // Set the motor's target position to 300 ticks

            // Switch to RUN_TO_POSITION mode

            // Start the motor moving by setting the max velocity to 200 ticks per second
            armMotor.setVelocity(250);
            telemetry.addData("velocity", armMotor.getVelocity());
            telemetry.addData("position", armMotor.getCurrentPosition());
            telemetry.addData("is at target", !armMotor.isBusy());
            telemetry.addData("armPosition",armPosition);
            telemetry.update();

            if (gamepad1.left_bumper) {
                sleep(5);
                armPosition = armPosition - 3;
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            } if (gamepad1.right_bumper) {
                sleep(5);
                armPosition = armPosition + 3;
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            }
            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            telemetry.addData("armPosition",armPosition);
            telemetry.update();
        }
    }
}