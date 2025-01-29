package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Arm {
    private DcMotorEx angleLeft = null;
    private DcMotorEx angleRight = null;

    private double startAngle = 0;

    private DigitalChannel limitSwich = null;

    private final double ANGLE_AMP_LIMIT = 0;

    private final double KF = 0;


    //extend

    private DcMotorEx extendLeft = null;

    private DcMotorEx extendRight = null;

    private final double EXTEND_UP_LIMIT_AMPS = 0;

    private final double EXTEND_DOWN_LIMIT_AMPS = 0;

    private final double MIN_EXTEND = 0;

    private final double LIMIT = 0;

    private boolean IS_DEBUG_MODE = false;
    public Arm(OpMode opMode, boolean isDebug){

        IS_DEBUG_MODE = isDebug;

        angleRight = opMode.hardwareMap.get(DcMotorEx.class, "AR");
        angleLeft = opMode.hardwareMap.get(DcMotorEx.class, "AL");

        extendRight = opMode.hardwareMap.get(DcMotorEx.class, "ER");
        extendLeft = opMode.hardwareMap.get(DcMotorEx.class, "EL");

        limitSwich = opMode.hardwareMap.get(DigitalChannel.class, "LS");

        init();

    }

    public void init(){
        angleLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        angleRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendRight.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoders();
    }

    public void resetEncoders(){
       resetExtendEncoders();
       resetAngleEncoders();
    }

    public void resetExtendEncoders(){
        extendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetAngleEncoders(){
        angleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getLeftAngle(){
        return angleLeft.getCurrentPosition() + startAngle;
    }

    public double getRightAngle(){
        return angleRight.getCurrentPosition() + startAngle;
    }

    public double getAngle(){
        return (getRightAngle()+ getLeftAngle())/2;
    }

    public double getMIN_EXTEND() {
        return MIN_EXTEND;
    }

    public double getLeftExtend(){
        return extendLeft.getCurrentPosition();
    }

    public double getRightExtend(){
        return extendRight.getCurrentPosition();
    }

    public double getExtend(){
        return (getLeftExtend()+ getRightExtend())/2;
    }

    public double getCurrentAngleLeft(){
        return angleLeft.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getCurrentAngleRight(){
        return angleRight.getCurrent(CurrentUnit.MILLIAMPS)
    }

    public double getCurrentAngle(){
        return (getCurrentAngleRight() + getCurrentAngleLeft())/2;
    }

    public double getCurrentExtendLeft(){
        return extendLeft.getCurrent(CurrentUnit.MILLIAMPS)
    }

    public double getCurrentExtendRight(){
        return extendRight.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getCurrentExtend(){
        return (getCurrentExtendLeft() + getCurrentExtendRight())/2;
    }

    public double calculatrF(){
        return (Math.cos(Math.toRadians(getAngle())) * KF) * ((getMIN_EXTEND() + getExtend())/ getMIN_EXTEND());
    }





    public void setAnglePower(double power){
        if (getCurrentAngle() >= ANGLE_AMP_LIMIT && power > 0){
            power = 0;
        }
        if (limitSwich.getState()){
            resetAngleEncoders();
            if (power < 0) {
                power = 0;
            }

        }

        angleRight.setPower(power);
        angleLeft.setPower(power);
    }

    public void setPowerAngleWithF(double power){
        double feedForward = calculatrF();
        setAnglePower(power+feedForward);
    }

    public void setPowerExtend (double power){
        if (getCurrentExtend() >= EXTEND_DOWN_LIMIT_AMPS && getCurrentExtend() <= EXTEND_UP_LIMIT_AMPS && power >0){
            power = 0;
        }
        else if (getCurrentExtend() >= EXTEND_UP_LIMIT_AMPS){
            resetExtendEncoders();
            if (power < 0){
                power = 0;
            }
        }
        extendLeft.setPower(power);
        extendRight.setPower(power);
    }

    public void setPowerExtendWithLimits(double power){
        if (getExtend() >= LIMIT && power > 0){
            power = 0;
        }
        setPowerExtend(power);
    }

    public class moveAngle implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }


    }

}