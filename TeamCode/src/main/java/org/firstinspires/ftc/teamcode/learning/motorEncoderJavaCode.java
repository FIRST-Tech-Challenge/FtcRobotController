package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap; //Importing data
class align extends Thread{
    private DcMotor motor;
    private double pos;
    public void init(DcMotor motor, double pos){
        this.motor = motor;
        this.pos = pos;
    }
    public void run(){
        boolean running = true;
        double speed = 0;
        while(running)
        {
            double currentPos = motor.getCurrentPosition();
            speed = currentPos / pos;
            if(speed < 0.2){
                speed = 0.2;
            }
            motor.setPower(pos > currentPos ? speed : -speed); // Decide if you are going backwards or forwards
            if(currentPos + 10 >= pos && currentPos - 10 <= pos) running = false; // Stop running if within 10 of the target
        }
    }
}
public class motorEncoderJavaCode
{
    private double ticksPerRotation; //Variable
    private DcMotor motor;

    public void init(HardwareMap hwMap, String motorName)
    {
        motor = hwMap.get(DcMotor.class, motorName); //Setting motor name to "motor"
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Setting mode to Run With Encoder
        ticksPerRotation = motor.getMotorType().getTicksPerRev(); //Getting Ticks per rotation/revolution
    }

    public void setMotorSpeed(double speed)
    {
        motor.setPower(speed);
    } //Setting motor speed
    public double getMotorRotations()
    {
       return motor.getCurrentPosition() / ticksPerRotation; //Getting current position
    }
    public void goToPosition(double pos)
    {
        align alignmanger = new align();
        alignmanger.init(motor,pos);
        Thread align = new Thread(alignmanger);
        align.run();
    }
}
