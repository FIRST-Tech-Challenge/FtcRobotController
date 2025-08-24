package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench1 {
    private DcMotor motor;
    private double ticksPerRev;

   public void init(HardwareMap hwMap)  {

        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed){
       motor.setPower(speed);
    }

    public double getMotorRevs(){
       return motor.getCurrentPosition() / ticksPerRev;
    }

}
