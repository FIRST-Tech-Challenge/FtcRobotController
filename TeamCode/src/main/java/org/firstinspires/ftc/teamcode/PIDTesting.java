package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDTesting extends LinearOpMode {

    DcMotorEx motor;
    /*
    need to tune these values with by using this helpful method from Victor Lomboda:
    Raise P until it oscillates around the setpoint. Add D to cancel out the oscillation. Only add I if you need to
    also we need to get some things like ftc dashboard
     */
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        motor =  hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            double power = PIDController(100, motor.getCurrentPosition());
            motor.setPower(power);

        }

    }

    // better to put this PID controller into a class
    public double  PIDController(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}
