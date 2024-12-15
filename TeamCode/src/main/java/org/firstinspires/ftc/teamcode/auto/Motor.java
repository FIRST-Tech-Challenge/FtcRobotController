package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor {
//    public enum MODE {
//        AUTO, POWER, POSITION;
//    }

    public String motorname;
    private DcMotor motor;
    private boolean isReversed;
    private boolean autonomous;
    public Motor(String name, boolean reversed, boolean autonomous){
        this.motorname = name;
        this.isReversed = reversed;
        this.autonomous = autonomous;
    }

    public void setupMotor(){
        if(isReversed){motor.setDirection(DcMotor.Direction.REVERSE);}
        if(autonomous){motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
    }
    public void setMotor(DcMotor motor) {
        this.motor = motor;
    }
    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
//        if (mode==MODE.POWER){
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            return;
//        }else if (mode==MODE.POSITION){
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            return;
//        } else if (mode==MODE.AUTO){
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            return;
//        }
    }

    public void move(int ticks) {
        int position = motor.getCurrentPosition() + ticks;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    public boolean isBusy(){
        return motor.isBusy();
    }
    public void stopMotor() {
        motor.setPower(0);
        motor.setTargetPosition(motor.getCurrentPosition());
    }

    public DcMotor getMotor() {
        return motor;
    }

}