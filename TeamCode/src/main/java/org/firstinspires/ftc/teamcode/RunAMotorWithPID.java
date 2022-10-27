package org.firstinspires.ftc.teamcode;

import android.text.style.LeadingMarginSpan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
public class RunAMotorWithPID extends LinearOpMode {
    DcMotor myMotor = null;

    double integralSum = 0.0;
    double Kp = 0.0; // These need to be tuned
    double Ki = 0.0;
    double Kd = 0.0;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0.0;

    @Override
    public void runOpMode(){
        myMotor = hardwareMap.get(DcMotor.class, "myMotor");
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            double power = PIDControl(100, myMotor.getCurrentPosition());
            myMotor.setPower(power);
        }
    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += (error * timer.seconds());
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
