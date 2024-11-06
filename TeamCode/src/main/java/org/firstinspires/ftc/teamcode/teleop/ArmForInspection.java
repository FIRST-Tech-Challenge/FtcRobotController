package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class ArmForInspection extends OpMode {
    DcMotorEx armMotor;
    int armPosition = 0;
    double ticks = 2786.2;
    double newTarget;

    @Override
    public void init() {
    // do nothing
    }

    @Override
    public void start() {
    // do nothing
    }

    @Override
    public void loop() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        // Reset the encoder during initialization
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setVelocity(200);
        armMotor.setTargetPosition(100);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        try {sleep(1500);} catch (Exception e) {
            // exception caught
        }
        armMotor.setTargetPosition(300);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("velocity", armMotor.getVelocity());
        telemetry.addData("position", armMotor.getCurrentPosition());
        telemetry.addData("is at target", armMotor.isBusy());
        telemetry.addData("armPosition", armPosition);
        telemetry.update();
    }
}