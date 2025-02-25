package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous
public class PIDmotor extends LinearOpMode {
    DcMotorEx backleft, backright, frontleft, frontright;
    double Kp = 0.15;
    double Ki = 0.5;
    double Kd = 0.2;
    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        backleft = hardwareMap.get(DcMotorEx.class, "BL");
        frontleft = hardwareMap.get(DcMotorEx.class, "FL");
        frontright = hardwareMap.get(DcMotorEx.class, "FR");
        backright = hardwareMap.get(DcMotorEx.class, "BR");
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            double power = PIDControl(100, backleft.getCurrentPosition());
            double power1 = PIDControl(100, frontleft.getCurrentPosition());
            double power2 = PIDControl(100, backright.getCurrentPosition());
            double power3 = PIDControl(100, frontright.getCurrentPosition());

            backleft.setPower(power);
            frontleft.setPower(power1);
            frontright.setPower(power3);
            backright.setPower(power2);
        }
    }
    public double PIDControl (double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = ((error * Kp) + (derivative * Kd) + (integralSum * Ki)/2);
        return output;
    }
}