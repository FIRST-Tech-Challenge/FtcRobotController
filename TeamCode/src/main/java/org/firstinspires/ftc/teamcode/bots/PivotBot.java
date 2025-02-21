package org.firstinspires.ftc.teamcode.bots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PivotBot extends OdometryBot { //change back to odometry bot later

    TelemetryPacket packet = new TelemetryPacket();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int samplePivotDropOffPos = -2050; // was 2150, seems to be going too far

    public static int sampleSlideDropOffPos = 780;// WAS 770, MADE LOWER TO AVOID CLAW GETTING CAUGHT ON BUCKET

    public final float ticksToDegree = 360/8192; // 8192 ticks per rotation
    public float kfAngled;
    public static double kp = 3.6;
    public static double ki = 0;
    public static double kd = 0.43;
    public static double kf = -0.72;

    PIDFController pivotController = new PIDFController(kp,ki,kd,kf);

    public static int maximumPivotPos = -1901;//1000; changed value to test climb
    public static int minumimPivotPos = 0;

    public static int maximumSlidePos = 850;
    public static int minimumSlidePos = 0;

    public static int slideTarget = 0;

    public boolean pivotOutOfRange = false;
    public static int pivotTarget = 0;
    public int oldPt = 0;
    public double pivotPower = 0.7;
    public boolean reachedPivotPos = false;
    public DcMotorEx pivotMotor2 = null;
    public DcMotorEx pivotMotor1 = null;
    public DcMotorEx slideMotor1 = null;
    public DcMotorEx slideMotor2 = null;

//    double integralSum = 0;
//    double Kp = 0.5;
//    double Ki = 0;
//    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public PivotBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pivotMotor1 = hwMap.get(DcMotorEx.class, "pivot1");
        pivotMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor1.setPower(0);

        pivotMotor2 = hwMap.get(DcMotorEx.class, "pivot2");
        pivotMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor2.setPower(0);

        slideMotor1 = hwMap.get(DcMotorEx.class, "slide1");
        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor1.setPower(0);

        slideMotor2 = hwMap.get(DcMotorEx.class, "slide2");
        slideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setPower(0);


        // TODO
    }

    public int getSlidePosition() {
        return slideMotor1.getCurrentPosition();
    }

    public int getPivotPosition() {
        return pivotMotor1.getCurrentPosition();
    }

    public float getSlideMotor1Power(){
        return (float) slideMotor1.getPower();
    }

    public float getSlideMotor2Power(){
        return (float) slideMotor2.getPower();
    }

    protected void onTick() {
        super.onTick();
        pivotController.setTolerance(5,10);
        pivotController.setSetPoint(pivotTarget);
        //elseF
            if (pivotMotor1.getCurrentPosition() > -2100) {
                kfAngled = (float) (kf * Math.cos(Math.toRadians(ticksToDegree * pivotMotor1.getCurrentPosition())));
            } else{ kfAngled = 0;}
            if(pivotMotor1.getCurrentPosition() < 0 && pivotTarget == 0){

            }
            else if(!pivotController.atSetPoint()) {
                pivotController.setPIDF(kp, ki, kd, 0);
                double output = pivotController.calculate(pivotMotor1.getCurrentPosition());
                pivotMotor2.setVelocity(output + kfAngled);
                pivotMotor1.setVelocity(output + kfAngled);
                telemetry.addData("output velocity", output);
                packet.put("position", pivotMotor1.getCurrentPosition());
                packet.put("target position", pivotTarget);
                dashboard.sendTelemetryPacket(packet);
            }
        else {
            pivotMotor1.setPower(0);
            pivotMotor2.setPower(0);
            pivotMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        /** commmented out pid to test slide */
//        pivotController.setTolerance(20,30);
//        kfAngled = (float) (kf*Math.cos(Math.toRadians(ticksToDegree)*(-pivotMotor1.getCurrentPosition())));
//        pivotController.setPIDF(kp,ki,kd,0);
//        pivotController.setSetPoint(pivotTarget);
//        int pivotCorrected = -pivotMotor1.getCurrentPosition();
//
//        if (!pivotController.atSetPoint()){
//            double output = pivotController.calculate(pivotMotor1.getCurrentPosition());
//            pivotMotor2.setVelocity(output+kfAngled);
//            pivotMotor1.setVelocity(output+kfAngled);
//            telemetry.addData("output velocity",output);
//        } else{
//            pivotMotor2.setPower(0);
//            pivotMotor1.setPower(0);
//        }


//        if (reachedPivotPos && oldPt != pivotTarget) {
//
//            reachedPivotPos = false;
//            oldPt = pivotTarget;
//
//        }
//
//        if (pivotTarget > minumimPivotPos - 100 && pivotTarget < maximumPivotPos + 100){
//
//            pivotOutOfRange = false;
//
//            pivotPower = PIDControl(pivotTarget, getPivotPosition());
//
//            runPivotMotors(pivotTarget, 0.3);
//
//            // TODO : PID control for the pivot motor
//
//        } else {
//
//            pivotOutOfRange = true;
//            pivotMotor1.setPower(0);
//            pivotMotor2.setPower(0);
//
//        }
        boolean slideMotorsNearBottom = (-70 < slideMotor1.getCurrentPosition() && slideMotor1.getCurrentPosition() < 0);
        if (slideMotorsNearBottom && (slideTarget == 0)){
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
            slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if (slideTarget > -1 && slideTarget < maximumSlidePos){
            slideMotor1.setTargetPosition(slideTarget);
            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setTargetPosition(slideTarget);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // TODO : PID control for the slide motor
            slideMotor1.setPower(1);
            slideMotor2.setPower(1);

        }
        else if (slideMotor1.getCurrentPosition() < 0){
            slideRunToPosition(0);
        }

    }
//    public void slideByDelta(int delta){
//        slideTarget += delta;
//    }
    public void slideControl(boolean up, boolean down) {
        if (up) {
            if (/**slideMotor1.getCurrentPosition() < maximumSlidePos*/true) {
                slideTarget = slideMotor1.getCurrentPosition() + ((maximumSlidePos - slideMotor1.getCurrentPosition()) / 10);
//                slideMotor1.setTargetPosition(slideTarget);
//                slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor2.setTargetPosition(slideTarget);
//                slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            }
        }
        if (down) {
            if (/**slideMotor1.getCurrentPosition() > minimumSlidePos*/ true) {
                slideTarget = slideMotor1.getCurrentPosition() - (slideMotor1.getCurrentPosition() / 10);
//                slideMotor1.setTargetPosition(slideTarget);
//                slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor2.setTargetPosition(slideTarget);
//                slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }

        //make pivot same
    }

    public void slidePosition(int position){
        slideTarget = position;
        slideMotor1.setTargetPosition(slideTarget);
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setTargetPosition(slideTarget);
        slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void pivotRunToPosition(int targetPos){
        pivotTarget = targetPos;
    }

    public void slideRunToPosition(int targetPos){
        slideTarget = targetPos;
    }
    public void pivotControl(boolean up, boolean down){
        pivotController.setTolerance(40,30);
        if (pivotMotor1.getCurrentPosition() > -20 && pivotTarget == 0 ){
            pivotMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pivotMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pivotMotor1.setPower(0);
            pivotMotor2.setPower(0);
        }
        else if(pivotMotor1.getCurrentPosition() > - 2000) {
            if (pivotMotor1.getCurrentPosition() > -1900) {
                kfAngled = (float) (kf * Math.cos(Math.toRadians(ticksToDegree * pivotMotor1.getCurrentPosition())));
            } else{ kfAngled = 0;}
            pivotController.setPIDF(kp, ki, kd, 0);
            pivotController.setSetPoint(pivotTarget);

                double output = pivotController.calculate(pivotMotor1.getCurrentPosition());
                pivotMotor2.setVelocity(output + kfAngled);
                pivotMotor1.setVelocity(output + kfAngled);
                telemetry.addData("output velocity", output);
            packet.put("position",pivotMotor1.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

        } else {
            pivotMotor1.setPower(0);
            pivotMotor2.setPower(0);
            pivotMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            pivotMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);}

//        pivotController.setTolerance(10,20);
//
//        pivotController.setSetPoint(pivotTarget);
//
//        while (!pivotController.atSetPoint()){
//            double output = pivotController.calculate(pivotMotor1.getCurrentPosition(),pivotTarget);
//            pivotMotor2.setVelocity(output);
//            pivotMotor1.setVelocity(output);
//        }


//        while (!pivotCoF)ntroller.atSetPoint()){
//            double output = pivotController.calculate(pivotMotor1.getCurrentPosition());
//            pivotMotor2.setVelocity(output);
//            pivotMotor1.setVelocity(output);
//        }
//        if (pivotController.atSetPoint()) {
//            pivotMotor2.setPower(0);
//            pivotMotor1.setPower(0);
//        }
//        if (up) {
//            if (pivotMotor1.getCurrentPosition() < maximumPivotPos - 100) {
//                pivotTarget = pivotMotor1.getCurrentPosition() + ((maximumPivotPos - pivotMotor1.getCurrentPosition()) / 10);
//            }
//        }
//        if (down) {
//            if (pivotMotor1.getCurrentPosition() > minumimPivotPos) {
//                pivotTarget = pivotMotor1.getCurrentPosition() - (pivotMotor1.getCurrentPosition() / 10);
//
//            }
//
//        }
       }
//    public void runPivotMotors(int pT, double power){
//        pivotMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pivotMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        if (getPivotPosition() > pT - 20 && getPivotPosition() < pT + 20) {
//
//            reachedPivotPos = true;
//
//            pivotMotor1.setPower(0.2);
//            pivotMotor2.setPower(0.2);
//
//        } else if (reachedPivotPos == false) {
//
//            if (getPivotPosition() < pT) {
//
//                pivotMotor1.setPower(power);
//                pivotMotor2.setPower(power);
//
//            }
//
//            if (getPivotPosition() > pT) {
//
//                pivotMotor1.setPower(-power);
//                pivotMotor2.setPower(-power);
//
//            }
//        }
//    }

//    public double PIDControl(double reference, double state)
//    {
//
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
//        return output;
//
//    }

//    public void climbControl(boolean input) {
//
//        int state = 0;
//        int extended = 1;
//        int retracted = 2;
//        if (input&&time.seconds()>0.2&&state!=extended) {
//            pivotTarget = 1900;
//            time.reset();
//            state = extended;
//        }
//
//        if (input&&time.seconds()>0.2&&state!=retracted){
//            pivotTarget = 100;
//        }
//    }
}