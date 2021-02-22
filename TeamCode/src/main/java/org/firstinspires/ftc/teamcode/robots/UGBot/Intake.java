package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public DcMotor intakeMotor = null;
    private double speed;
    boolean active;

    public Intake(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        speed = 0;
    }

    public void update(){
        if(active){
            intakeMotor.setPower(speed);
        }
    }

    //region getters and setters

    public void setIntakeSpeed(double speed){
        this.speed = speed;
    }

    public void setActive(boolean active){this.active = active;}

    private boolean fullTilt = false;
    public void toggleFullTilt(){
        fullTilt = !fullTilt;
        if(fullTilt){
            speed = 1;
        }
        else{
            speed = 0;
        }
    }

    //endregion


}
