package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;

public class Intake {

    public DcMotor intakeMotor = null;
    public Servo tiltServo = null;
    private double speed;
    private boolean active = true;
    private int tiltTargetPosition = 1300;



    public Intake(DcMotor intakeMotor, Servo tiltServo) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.tiltServo = tiltServo;
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

        tiltServo.setPosition(Conversions.servoNormalize(tiltTargetPosition));
    }

    //region getters and setters

    public void setIntakeSpeed(double speed){
        this.speed = speed;
    }

    public boolean setTiltTargetPosition(int tiltTargetPosition){this.tiltTargetPosition = tiltTargetPosition; return true;}

    public int getTiltTargetPosition(){return this.tiltTargetPosition;}

    public double getIntakeSpeed(){
        return speed;
    }

    public void setActive(boolean active){this.active = active;}

    private boolean fullTilt = false;
    public void toggleFullTilt(){
        fullTilt = !fullTilt;
        if(fullTilt){
            speed = 1;
            setTiltTargetPosition(Constants.INTAKE_SERVO_TRANSIT);
        }
        else{
            speed = 0;
            setTiltTargetPosition(Constants.INTAKE_SERVO_VERTICAL);
        }
    }

    //endregion


}
