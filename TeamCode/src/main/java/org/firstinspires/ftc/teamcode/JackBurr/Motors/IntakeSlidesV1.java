package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class IntakeSlidesV1 {
    public HardwareMap hardwareMap;
    public DcMotor intakeSlides;
    public double power = 0;
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.intakeSlides = hardwareMap.get(DcMotor.class,"intakeSlides");
        this.intakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeSlides.setPower(0);
        this.power = 0;
        this.intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intakeOut(){
        intakeSlides.setPower(1);
        intakeSlides.setTargetPosition(850);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void intakeIn(){
        intakeSlides.setPower(1);
        intakeSlides.setTargetPosition(135);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void intakeAway(){
        intakeSlides.setPower(1);
        intakeSlides.setTargetPosition(175);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void intakeAllTheWayIn(){
        intakeSlides.setPower(1);
        intakeSlides.setTargetPosition(40);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runToPosition(int position, double power){
        if(intakeSlides.getCurrentPosition() != position) {
            intakeSlides.setPower(power);
            this.power = power;
            intakeSlides.setTargetPosition(position);
            intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            intakeSlides.setPower(0);
            this.power = 0;
        }
    }

    public boolean isIn(){
        return getCurrentPosition() == 135;
    }
    public boolean isAllTheWayIn(){
        return getCurrentPosition() == 40;
    }

    public boolean isOut(){
        return getCurrentPosition() == 850;
    }

    public double getPower(){
        return this.power;
    }

    public int getCurrentPosition(){
        return intakeSlides.getCurrentPosition();
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour){
        intakeSlides.setZeroPowerBehavior(behaviour);
    }
}
