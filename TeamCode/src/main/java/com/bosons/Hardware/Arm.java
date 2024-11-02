package com.bosons.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    public OpMode opm;
    public Motor rightExtendoMotor;
    public Motor leftExtendoMotor;

    //Rotation motor and pid//
    public Motor rightRotationMotor;
    public Motor leftRotationMotor;

    public PIDController controller;
    public static double p = 0.0016, i = 0.01, d = 0.00005;
    public static double f = 0.15;
    public static int rotTarget = 0;
    private final double ticks_in_degree = 8192/360;
    //---------------------//
    public int Pips;//pips are actually encoder "ticks" and should probably be renamed as such for clarity
    public int MinPip = 5;
    public int MaxPip = 2185;
    public double Power = 0.0;

    public Arm(OpMode op, double power){
        opm = op;
        Power = power;

        rightExtendoMotor = new Motor("RightExt",op);//op.hardwareMap.get(DcMotor.class, "RightArm");
        rightExtendoMotor.setTargetPosition(0);
        rightExtendoMotor.setConstants(DcMotor.RunMode.RUN_TO_POSITION,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.REVERSE);
        rightExtendoMotor.resetEncoder();

        leftExtendoMotor = new Motor("LeftExt",op);
        leftExtendoMotor.setTargetPosition(0);
        leftExtendoMotor.setConstants(DcMotor.RunMode.RUN_TO_POSITION,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.REVERSE);
        leftExtendoMotor.resetEncoder();



        rightRotationMotor = new Motor("RightRot",op);
        rightRotationMotor.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.FORWARD);

        leftRotationMotor = new Motor("LeftRot",op);
        leftRotationMotor.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.REVERSE);

        controller = new PIDController(p,i,d);
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void updatePidLoop(){
        controller.setPID(p,i,d);
        int armPos = leftRotationMotor.getCurrentPosition();
        double pid = controller.calculate(armPos,rotTarget);
        double ff = Math.cos(Math.toRadians(rotTarget/ticks_in_degree))*f;

        double power = pid + ff;

        if(armPos>100||rotTarget>100) {
            rightRotationMotor.setPower(power);
            leftRotationMotor.setPower(power);
        }
        else {
            rightRotationMotor.setPower(0);
            leftRotationMotor.setPower(0);
        }
        opm.telemetry.addData("pos ",armPos);
        opm.telemetry.addData("target ",rotTarget);
    }

    public void setPositionPolar(double r,double theta){
        //convert r (cm) to extension
        int ticks_per_cm = 0;
        int extensionTicks = (int)(r*ticks_per_cm);
        if(extensionTicks>MaxPip){extensionTicks=MaxPip;}
        //convert theta (cm) to encoder
        int rotationTicks = (int)(theta*ticks_in_degree);
        if(rotationTicks>2700){rotationTicks=2700;}
        //set target positions
        extendToTarget(extensionTicks);
        rotTarget = rotationTicks;
    }

    public void setPositionCartesian(double x, double y){
        setPositionPolar(Math.sqrt((x*x)+(y*y)),Math.atan(y/x));
    }

    public void extendToTarget(int Counts){
        //Dynamic Safety range
        /*
        if ((Pips - 5) < 0) {MinPip = 5;}
        else {MinPip = Pips - 5;}
        if ((Pips+5) > 2190) {MaxPip = 2185;}
        else {MaxPip = Pips + 5;}
        */
        //if pips is greater or smaller than the arms maximums then clamp to arm maximums and dont exceed them.
        if(Counts <= MinPip) {// when arm is set to close
            if (rightExtendoMotor.getCurrentPosition() <= MinPip) {rightExtendoMotor.setPower(0);}
            else {rightExtendoMotor.setPower(Power);} // check if RightArm is fully closed then turn off power
            if(leftExtendoMotor.getCurrentPosition() <= MinPip){leftExtendoMotor.setPower(0);}
            else{leftExtendoMotor.setPower(Power);} // check if LeftArm is fully closed then turn off power
        }

        if(Counts >= MaxPip){
            if(rightExtendoMotor.getCurrentPosition() >= MaxPip){rightExtendoMotor.setPower(0);}
            else{rightExtendoMotor.setPower(Power);} // check if RightArm is fully extended then turn off power
            if(leftExtendoMotor.getCurrentPosition() >= MaxPip-MinPip){leftExtendoMotor.setPower(0);}
            else{leftExtendoMotor.setPower(Power);} // check if LeftArm is fully extended then turn off power
        }

        rightExtendoMotor.setTargetPosition(Counts);
        leftExtendoMotor.setTargetPosition(Counts);
    }
    public void MotorCheck(){
        if(Pips <= MinPip) {// when arm is set to close
            if (rightExtendoMotor.getCurrentPosition() <= MinPip && rightExtendoMotor.getTargetPosition()<= MinPip) {rightExtendoMotor.setPower(0);} // check if LeftArm is fully closed then turn off power
            else{rightExtendoMotor.setPower(Power);}
            if (leftExtendoMotor.getCurrentPosition() <= MinPip && leftExtendoMotor.getTargetPosition()<= MinPip) {leftExtendoMotor.setPower(0);} // check if LeftArm is fully closed then turn off power
            else{leftExtendoMotor.setPower(Power);}
        }
        if(Pips >= MaxPip){
            if(rightExtendoMotor.getCurrentPosition() >= MaxPip && rightExtendoMotor.getTargetPosition()<= MinPip){rightExtendoMotor.setPower(0);}   // check if RightArm is fully extended then turn off power
            else{rightExtendoMotor.setPower(Power);}
            if(leftExtendoMotor.getCurrentPosition() >= MaxPip && leftExtendoMotor.getTargetPosition()<= MinPip){leftExtendoMotor.setPower(0);}   // check if RightArm is fully extended then turn off power
            else{leftExtendoMotor.setPower(Power);}
        }
        //opm.telemetry.addData("R power ", rightExtendoMotor.getPower());
        //opm.telemetry.addData("L power ", leftExtendoMotor.getPower());
        //opm.telemetry.addData("R pos ", rightExtendoMotor.getCurrentPosition());
        //opm.telemetry.addData("L pos ", leftExtendoMotor.getCurrentPosition());
        //opm.telemetry.addData("R target ", rightExtendoMotor.getTargetPosition());
        //opm.telemetry.addData("L target ", leftExtendoMotor.getTargetPosition());
    }
    public void Stop(){
        rightExtendoMotor.setPower(0);
        leftExtendoMotor.setPower(0);
    }
}
