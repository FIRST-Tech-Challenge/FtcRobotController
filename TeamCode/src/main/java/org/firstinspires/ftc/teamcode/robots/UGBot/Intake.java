package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public DcMotor intakeMotor = null;
    public DcMotor tiltMotor = null;
    private double speed;
    private boolean active = true;
    private int tiltPosition = 0;


    public Intake(DcMotor intakeMotor, DcMotor tiltMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.tiltMotor = tiltMotor;
        this.tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed = 0;
    }

    public void update(){
        if(active){
            intakeMotor.setPower(speed);
            //tiltMotor.setTargetPosition(tiltPosition);
        }
        else{
            intakeMotor.setPower(0);
        }
    }

    //region getters and setters

    public void setIntakeSpeed(double speed){
        this.speed = speed;
    }

    public void setTiltPosition(int positionTics){this.tiltPosition = positionTics;}

    public int getTiltPosition(){return this.tiltPosition;}

    public int getTiltPositionActual(){return tiltMotor.getCurrentPosition();}

    public double getIntakeSpeed(){
        return speed;
    }

    public void setActive(boolean active){this.active = active;}

    private boolean fullTilt = false;
    public void toggleFullTilt(){
        fullTilt = !fullTilt;
        if(fullTilt){
            speed = -1;
        }
        else{
            speed = 0;
        }
    }

    //endregion


}
