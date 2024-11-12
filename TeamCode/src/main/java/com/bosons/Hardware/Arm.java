package com.bosons.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    //opMode for telemetry and hardware map;
    public OpMode opm;

    //arm extension motors
    public Motor rightExtendoMotor;
    public Motor leftExtendoMotor;

    //Wrist/intake servos
    private final Servo wristServo;
    private final CRServo intakeServo;

    //Rotation motor and pid//
    public Motor rightRotationMotor;
    public Motor leftRotationMotor;

    public PIDController controller;
    public static double p = 0.0016, i = 0.01, d = 0.00005;
    public static double f = 0.15;
    public static int rotTarget = 0;
    //---------------------//
    public static int extensionTarget = 0;
    public int acceptableExtensionError = 30;
    public int maxExtensionTicks = 2185;
    public double Power;

    //Constants
    private final double ticks_in_degree = (8192/360.0);
    private final double ticks_per_cm = (2190/48.96);

    /*
    Motion Profiling bullshit
    private double slope;
    private double radius_0;
    private double theta_0;
    */

    //Motion Smoothing;
    private int thetaTicksInitial;
    private int thetaTicks;
    private final ElapsedTime smoothingTimer = new ElapsedTime();
    pose bucketHigh;
    pose bucketLow;
    pose specimenHigh;
    pose specimenLow;
    pose intake;
    pose intakeStandby;
    public pose home;
    private double timeSlope;


    public Arm(OpMode op, double power){
        opm = op;
        Power = power;

        bucketHigh =  new pose(84.6,90,0.3);
        bucketLow  =  new pose(65,90,0.3);
        specimenHigh =  new pose(84.6,90,0.3);//REPLACE THE NUMBERS WITH THE RIGHT ONES
        specimenLow =  new pose(84.6,90,0.3);//REPLACE THE NUMBERS WITH THE RIGHT ONES
        intake = new pose(65,-6,0.9);

        intakeStandby = new pose(65,15,0.5);
        home = new pose(40.8,-28,0);



        rightExtendoMotor = new Motor("RightExt",op);
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

        wristServo = op.hardwareMap.get(Servo.class,"wrist");
        intakeServo = op.hardwareMap.get(CRServo.class,"intake");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public enum Height{
        High,
        Low
    }
    public enum Mode{
        Bucket,
        Specimen,
        Intake
    }



    public void setWristServo(double pos){
        wristServo.setPosition(pos);
    }

    public void setIntakePower(double power){
        intakeServo.setPower(power);
    }

    public void updatePidLoop(int target){
        controller.setPID(p,i,d);
        int armPos = leftRotationMotor.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f;

        double power = (pid + ff);//*0.5;

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

    public void setPositionPolarSmooth(pose P, double seconds){//Slows down the movement over a set time;
        //convert r (cm) to ticks
        extensionTarget = (int)((P.radius-40.8)*ticks_per_cm);//subtract the fixed length of the arm
        if(extensionTarget>maxExtensionTicks){extensionTarget=maxExtensionTicks;}
        if(extensionTarget<0){extensionTarget=0;}
        //convert theta (degrees) to ticks
        thetaTicks = (int)((P.theta+28)*ticks_in_degree);//subtract the initial -28 degree position of the arm
        if(thetaTicks>2700){thetaTicks=2700;}
        if(thetaTicks<0){thetaTicks=0;}

        //set the change in ticks over the set interval
        thetaTicksInitial = leftRotationMotor.getCurrentPosition();
        timeSlope = (thetaTicks-thetaTicksInitial)/(seconds*1000);//ticks per millisecond
        smoothingTimer.reset();
    }


    public void updatePositionSmooth(){
        if(rotTarget!=thetaTicks) {
            // initial position +/- ticksPerMillisecond*millisecond
            rotTarget = (int) (thetaTicksInitial + timeSlope * smoothingTimer.milliseconds());
            if(rotTarget>2700){rotTarget=2700;}
            if(rotTarget<0){rotTarget=0;}
            opm.telemetry.addData("smoothing: Target = ",rotTarget);
        }

        if((timeSlope>0 && rotTarget>thetaTicks)||(timeSlope<0 && rotTarget<thetaTicks)||(timeSlope==0)){
            rotTarget = thetaTicks;
        }//prevent any overshooting

        updatePosition();
    }

    public boolean isSmoothing(){
        opm.telemetry.addData("rotTarget = ",rotTarget);
        opm.telemetry.addData("thetaTicks",thetaTicks);
        return rotTarget != thetaTicks;
    }

    public void setPositionPolar(double r,double theta){
        //convert r (cm) to extension
        int extensionTicks = (int)((r-40.8)*ticks_per_cm);//subtract the fixed length of the arm
        if(extensionTicks>maxExtensionTicks){extensionTicks=maxExtensionTicks;}
        //convert theta (cm) to encoder
        int rotationTicks = (int)((theta+28)*ticks_in_degree);//subtract the initial -28 degree position of the arm
        if(rotationTicks>2700){rotationTicks=2700;}
        //set target positions
        extensionTarget = extensionTicks;
        rotTarget = rotationTicks;
    }

    public void updatePosition(){
        extendToTarget(extensionTarget,0.5);
        updatePidLoop(rotTarget);
        opm.telemetry.addData("armAngle ", getArmAngle());
        opm.telemetry.addData("armLength ",getArmLength());
    }

    public double getArmAngle(){
        return (leftRotationMotor.getCurrentPosition()/ticks_in_degree)-28;
    }
    public void resetArmAngle(){
        leftRotationMotor.resetEncoder();
    }
    public double getArmLength(){
        return ((leftExtendoMotor.getCurrentPosition()+rightExtendoMotor.getCurrentPosition())/2.0)/(ticks_per_cm)+40.8;
    }

    public void resetArmLength(){
        leftExtendoMotor.resetEncoder();
        rightExtendoMotor.resetEncoder();
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

    public void positionArm(Mode M, Height H, double T){
        switch (M){
            case Bucket:{
                switch (H){
                    case High:{
                        setPositionPolarSmooth(bucketHigh,T);
                        setWristServo(bucketHigh.wrist);
                        return;
                    }
                    case Low:{
                        setPositionPolarSmooth(bucketLow,T);
                        setWristServo(bucketLow.wrist);
                        return;
                    }
                }
                return;
            }
            case Specimen:{
                switch (H){
                    case High:{
                        setPositionPolarSmooth(specimenHigh,T);
                        setWristServo(specimenHigh.wrist);
                        return;
                    }
                    case Low:{
                        setPositionPolarSmooth(specimenLow,T);
                        setWristServo(specimenLow.wrist);
                        return;
                    }
                }
                return;
            }
            case Intake:{
                switch (H){
                    case High:{
                        setPositionPolarSmooth(intakeStandby,T);
                        setWristServo(intakeStandby.wrist);
                        return;
                    }
                    case Low:{
                        setPositionPolarSmooth(intake,T);
                        setWristServo(intake.wrist);
                        return;
                    }
                }
            }
        }
    }

    public static class pose {
        public double theta;
        public double radius;
        public double x;
        public double y;
        public double wrist;
        public pose(double r, double angle, double wristServoPosition){
            radius = r;
            theta = angle;
            wrist = wristServoPosition;
            x = radius*Math.cos(theta);
            y = radius*Math.sin(theta);
        }
    }
}
