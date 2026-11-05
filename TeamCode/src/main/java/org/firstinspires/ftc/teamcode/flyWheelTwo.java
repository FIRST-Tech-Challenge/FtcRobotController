package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class flyWheelTwo {

    private DcMotor motor;

    private double ticksPerRotation;


    public  void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, "flyWheel2");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
        motor.getPower();
    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }

    public double getMotorRotations() {
        return motor.getCurrentPosition() / ticksPerRotation;
    }
}
