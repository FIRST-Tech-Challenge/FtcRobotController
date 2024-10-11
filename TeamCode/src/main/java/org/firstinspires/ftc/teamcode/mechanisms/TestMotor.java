package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestMotor extends TestItem {
    private double speed;
    private DcMotor motor;

    public TestMotor(String description, double speed, DcMotor motor){
        super(description);
        this.speed = speed;
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void run(boolean on, Telemetry telemetry){
        int tgtPos = 2000;
        if (on){
            motor.setTargetPosition(tgtPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(speed);
        } else {
            motor.setPower(0.0);
            if (motor.getCurrentPosition() > tgtPos) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        telemetry.addData("Encoder Runs to 2000", motor.getCurrentPosition());
    }
}
