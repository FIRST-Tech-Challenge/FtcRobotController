package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
public class RunToPositionExample extends LinearOpMode {
    DcMotorEx armMotor;
    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        // Reset the encoder during initialization
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Set the motor's target position to 300 ticks
        armMotor.setTargetPosition(100);

        // Switch to RUN_TO_POSITION mode
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        armMotor.setVelocity(25);

        // Loop while the motor is moving to the target
        while(armMotor.isBusy() && !isStopRequested()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            telemetry.addData("velocity", armMotor.getVelocity());
            telemetry.addData("position", armMotor.getCurrentPosition());
            telemetry.addData("is at target", !armMotor.isBusy());
            telemetry.update();
        }
    }
}