package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.Conversions;

public class Intake {

    public DcMotor intakeMotor = null;
    public Servo tiltServo = null;
    public Servo outServo = null;
    private double speed;
    private boolean active = true;
    private int tiltTargetPosition = Constants.INTAKE_TILT_INIT_POS;
    private int outTargetPos = Constants.INTAKE_OUT_SERVO_IN;



    public Intake(DcMotor intakeMotor, Servo tiltServo, Servo outServo) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.tiltServo = tiltServo;
        this.outServo = outServo;
        speed = 0;
    }


    public void update(){
        if(active){
            intakeMotor.setPower(speed);
        }
        else{
            intakeMotor.setPower(0);
        }


        tiltServo.setPosition(Conversions.servoNormalize(tiltTargetPosition));
        outServo.setPosition(Conversions.servoNormalize(outTargetPos));
    }

    //region getters and setters

    public void setIntakeSpeed(double speed){
        this.speed = speed;
    }

    public boolean setTiltTargetPosition(int tiltTargetPosition){this.tiltTargetPosition = tiltTargetPosition; return true;}

    public int getTiltTargetPosition(){return this.tiltTargetPosition;}

    public boolean setOutTargetPosition(int outTargetPosition){this.outTargetPos = outTargetPosition; return true;}

    public int getOutTargetPosition(){return this.outTargetPos;}

    public double getIntakeSpeed(){
        return speed;
    }

    public void setActive(boolean active){this.active = active;}

    //endregion


}
