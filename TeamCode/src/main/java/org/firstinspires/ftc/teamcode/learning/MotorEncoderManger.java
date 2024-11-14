package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorEncoderManger
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
    public void goToPosition(double pos)
    {
//        boolean running = true;
//        while(running)
//        {
//            double currentPos = motor.getCurrentPosition();
//            double speed = 0.75;
//            if(currentPos/pos <= 1.0/4.0){ // If current pos is less than 1/4 to the target pos
//                speed = 0.25; // speed up
//            } else if(currentPos/pos <= 1.0/3.0) { // If current pos is less than 1/3 to the target pos
//                speed = 0.5; // speed up
//            } else if(currentPos/pos <= 1.0/2.0){ // If current pos is less than 1/2 to the target pos
//                speed = 0.65; // speed up
//            }
//            if(pos+10.0>currentPos && pos-10.0 < currentPos){ // if pos is within 20 ticks of the targeted pos
//                // Suggesting if(Math.round(pos / 10) * 10 == Math.round(currentPos / 10) * 10){
//                motor.setPower(pos < motor.getCurrentPosition() ? speed : -speed);
//            }else{
//                running = false;
//            }
//        }

        // Alternative code
        boolean running = true;
        double speed = 0;
        while(running)
        {
            double currentPos = motor.getCurrentPosition();
            speed = currentPos / pos < 0.5 ? 2 * currentPos / pos * 0.6 + 0.2 : speed; // Speed up until you are half of the way
            motor.setPower(pos < motor.getCurrentPosition() ? speed : -speed);
            if(Math.round(currentPos / 10) * 10 == Math.round(pos / 10) * 10) running = false;
        }

    }


}
