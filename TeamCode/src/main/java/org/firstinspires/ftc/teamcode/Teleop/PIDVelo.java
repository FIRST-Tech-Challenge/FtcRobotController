package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelo extends LinearOpMode {
    double p = 0, i = 0, d = 0;
    double integralSum = 0, lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double power = Controller(100, motor.getVelocity(), p, i, d);
            motor.setPower(power);
        }
    }


    public double Controller(double reference, double state, double Kp, double Ki, double Kd) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) - timer.seconds();
        lastError = error;
        double output = (error * p) + (derivative * d) + (integralSum * i);

        return output;
    }
}
