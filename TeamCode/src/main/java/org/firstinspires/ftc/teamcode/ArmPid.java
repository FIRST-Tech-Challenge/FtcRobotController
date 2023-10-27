package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Disabled
@Autonomous(name = "PIDARM")
public class ArmPid extends LinearOpMode {
    private DcMotorEx Arm;

    double integral = 0;
    double repetitions = 0;

    public static PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    FtcDashboard dashboard;

    public static double TARGET_POS = 100;

    ElapsedTime PIDTimer = new ElapsedTime();

    @Override
    public void runOpMode(){
        Arm = hardwareMap.get(DcMotorEx.class,"Arm");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboard = FtcDashboard.getInstance();



        waitForStart();

        moveArmMotor(TARGET_POS);
    }
    void moveArmMotor(double targetPosition){
        double error = Arm.getCurrentPosition();
        double lastError = 0;
        while(Math.abs(error)<= 9 && repetitions < 10){
            error = Arm.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError /PIDTimer.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            Arm.setPower(P + I + D);
            error = lastError;
            repetitions ++;
            PIDTimer.reset();
        }
    }
}