package org.firstinspires.ftc.teamcode.Drivetrain.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;

public class kevinMotor extends DcMotorExComposition {
    private double lastPower = 0;
    DcMotor motor = null;
    private double motorTolerance = 0.000001;

    public kevinMotor(DcMotor motor) {
        this.motor = motor;
    }


    public void setPower(double newPower){
        if(Math.abs(newPower - lastPower) > motorTolerance){
            motor.setPower(newPower);
            lastPower = newPower;
        }
    }



    public double getPower(){
        return lastPower;
    }
    public double getMotorTolerance(){
        return motorTolerance;
    }
    public void setTolerance() {
        motorTolerance = 0.000001;
    }


}