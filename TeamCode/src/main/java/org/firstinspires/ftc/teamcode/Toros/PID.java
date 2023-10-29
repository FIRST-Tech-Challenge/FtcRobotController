package org.firstinspires.ftc.teamcode.Toros;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "PID")
public class PID extends LinearOpMode {
    private DcMotorEx FrontLeftMotor;
    double flCurrentVel = 0;
    double FrontLeftmax = 0;
    private DcMotorEx BackLeftMotor;
    double BackLeftmax = 0;
    double blCurrentVel = 0;
    private DcMotorEx FrontRightMotor;
    double frCurrentVel = 0;
    double FrontRightMax = 0;
    private DcMotorEx BackRightMotor;
    double BackRightmax = 0;
    double brCurrentVel = 0;

    private double motoTargetVel = 800.0;
    private double resultMax = 2800.0;
    private double F = 32767.0/ resultMax;
    private double kP = F*0.1;
    private double kI =kP * 0.1;
    private double kD = kP * 0.01;
    private double position = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while(!isStarted()){
            telemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            runBl(motoTargetVel);
            runBr(motoTargetVel);
            runFr(motoTargetVel);
            runFl(motoTargetVel);
        }
    }public void initHardware(){
        initmotors(kP,kI,kD,F,position);
    }public void runBr(double velocity){
        BackRightMotor.setVelocity(velocity);
        brCurrentVel = BackLeftMotor.getVelocity();
        if(brCurrentVel > BackRightmax){
            BackRightmax = brCurrentVel;
        }
    } public void runFr(double velocity){
        FrontRightMotor.setVelocity(velocity);
        frCurrentVel = FrontRightMotor.getVelocity();
        if(frCurrentVel > FrontRightMax){
            FrontRightMax = frCurrentVel;
        }
} public void runBl(double velocity){
        BackLeftMotor.setVelocity(velocity);
        blCurrentVel = BackLeftMotor.getVelocity();
        if(blCurrentVel > BackLeftmax){
            BackLeftmax = blCurrentVel;
        }
    }public void runFl(double velocity){
        FrontRightMotor.setVelocity(velocity);
        flCurrentVel = FrontLeftMotor.getVelocity();
        if(flCurrentVel > FrontLeftmax){
            FrontLeftmax = frCurrentVel;
        }



    }public void initmotors(double kP, double kI, double kD, double F, double position){
        FrontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotorEx.class, "BackRightMotor");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FrontLeftMotor.setVelocityPIDFCoefficients(kP,kI,kD,F);
        BackRightMotor.setVelocityPIDFCoefficients(kP,kI,kD,F);
        FrontRightMotor.setVelocityPIDFCoefficients(kP,kI,kD,F);
        BackLeftMotor.setVelocityPIDFCoefficients(kP,kI,kD,F);

        FrontLeftMotor.setPositionPIDFCoefficients(position);
        FrontRightMotor.setPositionPIDFCoefficients(position);
        BackLeftMotor.setPositionPIDFCoefficients(position);
        BackRightMotor.setPositionPIDFCoefficients(position);

        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }public void telemetry(){
        telemetry.addData("Power",FrontLeftMotor.getPower());
        telemetry.addData("Power",FrontRightMotor.getPower());
        telemetry.addData("Power",BackLeftMotor.getPower());
        telemetry.addData("Power",BackRightMotor.getPower());

        telemetry.addData("Target velocity",flCurrentVel);
        telemetry.addData("Target velocity",frCurrentVel);
        telemetry.addData("Target velocity",blCurrentVel);
        telemetry.addData("Target velocity",brCurrentVel);

        telemetry.addData("F",F);
        telemetry.addData("kp",kP);
        telemetry.addData("kI",kI);
        telemetry.addData("kD",kD);

    }
}