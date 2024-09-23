package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tatooine.utils.PIDFController;

public class Lift {
    DcMotorEx liftMotor = null;
    private boolean didntFinishedHanging = true;
    private double power = 0;


    public Lift (HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    }

    public void init(){
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();
    }

    public Action hanging(){
        if(liftMotor.getCurrent(CurrentUnit.AMPS)<=5 && power<0) {
            resetEncoders();
        }
        return new setPowerAction();
    }
    public void resetEncoders(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }

    public void setLiftMotor(DcMotorEx liftMotor) {
        this.liftMotor = liftMotor;
    }

    public boolean isDidntFinishedHanging() {
        return didntFinishedHanging;
    }

    public void setDidntFinishedHanging(boolean didntFinishedHanging) {
        this.didntFinishedHanging = didntFinishedHanging;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public class setPowerAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            while (liftMotor.getCurrentPosition()>=0) {
                liftMotor.setPower(-1);
            }
            while (liftMotor.getCurrentPosition()<=0){
                liftMotor.setPower(1);
            }
            liftMotor.setPower(-1);
                return true;
        }
    }
}
