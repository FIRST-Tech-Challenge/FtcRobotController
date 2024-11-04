package com.bosons.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public int extensionTarget = 0;
    public int acceptableExtensionError = 30;
    public int maxExtensionTicks = 2185;
    public double Power = 0.0;

    private double ticks_per_cm = (2190/48.96);

    private double slope;
    private double radius_0;
    private double theta_0;



    public Arm(OpMode op, double power){
        opm = op;
        Power = power;

        rightExtendoMotor = new Motor("RightExt",op);//op.hardwareMap.get(DcMotor.class, "RightArm");
        rightExtendoMotor.setTargetPosition(0);
        rightExtendoMotor.setConstants(DcMotor.RunMode.RUN_TO_POSITION,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.REVERSE);

        leftExtendoMotor = new Motor("LeftExt",op);
        leftExtendoMotor.setTargetPosition(0);
        leftExtendoMotor.setConstants(DcMotor.RunMode.RUN_TO_POSITION,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.FORWARD);



        rightRotationMotor = new Motor("RightRot",op);
        rightRotationMotor.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.FORWARD);

        leftRotationMotor = new Motor("LeftRot",op);
        leftRotationMotor.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER,DcMotor.ZeroPowerBehavior.BRAKE,DcMotor.Direction.REVERSE);

        controller = new PIDController(p,i,d);
        opm.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void updatePidLoop(int target){
        controller.setPID(p,i,d);
        int armPos = leftRotationMotor.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f;

        double power = pid + ff;

        if(armPos>100||target>100) {
            rightRotationMotor.setPower(power);
            leftRotationMotor.setPower(power);
        }
        else {
            rightRotationMotor.setPower(0);
            leftRotationMotor.setPower(0);
        }
        opm.telemetry.addData("pos ",armPos);
        opm.telemetry.addData("target ",target);
    }

    public void setPositionPolar(double r,double theta){
        //convert r (cm) to extension
        int extensionTicks = (int)((r-38.4)*ticks_per_cm);//subtract the fixed length of the arm
        if(extensionTicks>maxExtensionTicks){extensionTicks=maxExtensionTicks;}
        //convert theta (cm) to encoder
        int rotationTicks = (int)((theta+28)*ticks_in_degree);//subtract the initial -28 degree position of the arm
        if(rotationTicks>2700){rotationTicks=2700;}
        //set target positions
        extensionTarget = extensionTicks;
        rotTarget = rotationTicks;

        radius_0 = getArmLength();
        theta_0 = getArmAngle();
        slope = (r-radius_0)/(theta-theta_0);
    }

    public void updatePosition(){
        extendToTarget(extensionTarget,0.5);
        //r=r_0+m(t-t_0)
        double incrementalTheta = -(Math.toRadians(theta_0)+(getArmLength()-radius_0)/slope);
        int incrementalTicks = (int)(incrementalTheta*ticks_in_degree);
        updatePidLoop(incrementalTicks);
        opm.telemetry.addData("inc_theta ",incrementalTheta);
    }

    public double getArmAngle(){
        return (leftRotationMotor.getCurrentPosition()/ticks_per_cm)-28;
    }
    public double getArmLength(){
        return ((leftExtendoMotor.getCurrentPosition()+rightExtendoMotor.getCurrentPosition())/2.0)/(ticks_per_cm)+38.4+2.4;
    }

    public void setPositionCartesian(double x, double y){
        setPositionPolar(Math.sqrt((x*x)+(y*y)),Math.atan(y/x));
    }

    public void extendToTarget(int counts, double power){
        //Clamp incoming target to limits
        if(counts<0){counts=0;}
        if(counts>maxExtensionTicks){counts=maxExtensionTicks;}
        //set target position
        rightExtendoMotor.setTargetPosition(counts);
        leftExtendoMotor.setTargetPosition(counts);
        //set power if it wont burn the motor
        if(!rightExtendoMotor.burnCheck(acceptableExtensionError)){
            rightExtendoMotor.setPower(power);
        }
        if(!leftExtendoMotor.burnCheck(acceptableExtensionError)){
            leftExtendoMotor.setPower(power);
        }
    }

    public void Stop(){
        rightExtendoMotor.setPower(0);
        leftExtendoMotor.setPower(0);
    }

    public enum postition{
        topBucket
    }
}
