package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorEncodersManger
{
    private double ticksPerRotation;
    private DcMotor motor;

    public void init(HardwareMap hwMap, String motorName)
    {
        motor = hwMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed)
    {
        motor.setPower(speed);
    }
    public double getMotorRotations()
    {
       return motor.getCurrentPosition() / ticksPerRotation;
    }
    public void goToPosition(){

    }
}
