package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Input {

    Motors motors;
    Servos servos;
    ElapsedTime elapsedTime;

//    FtcDashboard dashboard;
//    Telemetry dashboardTelemetry;

    public Input(HardwareMap hardwareMap) {
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
        elapsedTime = new ElapsedTime();

//        dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
        //setPoint = motors.getArmPosition();
    }

    public void move(double power) {

        motors.MoveMotor(Motors.Type.LeftBack, -power);
        motors.MoveMotor(Motors.Type.LeftFront, -power);
        motors.MoveMotor(Motors.Type.RightFront, -power);
        motors.MoveMotor(Motors.Type.RightBack, -power);

    }

    public void strafe(double power) {
        motors.MoveMotor(Motors.Type.LeftFront, power); // left front
        motors.MoveMotor(Motors.Type.RightFront, -power); // right front

        motors.MoveMotor(Motors.Type.LeftBack, -power); // left back
        motors.MoveMotor(Motors.Type.RightBack, power); // right back
    }

    public void spin(double power) {
        motors.MoveMotor(Motors.Type.LeftFront, power); // left front
        motors.MoveMotor(Motors.Type.LeftBack, power); // left back

        motors.MoveMotor(Motors.Type.RightFront, -power); // right front
        motors.MoveMotor(Motors.Type.RightBack, -power); // right back
    }

    public void claw(boolean grabButton, boolean releaseButton) {

        if (releaseButton) {
            servos.moveServo(Servos.Type.Claw, 270);
        }
        else if (grabButton) {
            servos.moveServo(Servos.Type.Claw, 0);
        }
    }

    public void upArm(double power) {
        double thing = (-(Math.abs(motors.getArmPosition() - 440)) / 5.6) + 100;

        double restingUpArm = motors.getUpArmPosition();

        double realPower = Math.max(restingUpArm, Math.min(power, thing));
        motors.MoveMotor(Motors.Type.Pull, realPower);
    }





    // PID variables
//    public static double kp = 2;  // Proportional gain
//    public static double ki = 0.45;  // Integral gain
//    public static double kd = 0.13;  // Derivative gain
//
//    public int setPoint;

//    double prevError = 0;  // Previous error, used for derivative
//    double integral = 0;   // Integral term


//    public void ArmPidControl(double deltaTime, double input) {
//
//        setPoint += (int) (input * 100);    //multiply the game pad input by 100 so that there are no decimals which doesn't work in the setPoint then turn it into and int
//
//        // Time difference (dt)
//        double dt = deltaTime;
//
//        double processValue = motors.getArmPosition(); // Finding the position of the motors
//
//        // Calculate error
//        double errorValue = setPoint - processValue;
//
//
//        // Prevent divide-by-zero errors
//        if (dt == 0) {
//            dt = 0.1;  // Default small value if no time has passed
//        }
//
//        // Calculate PID terms
//        double proportional = kp * errorValue;
//
//        integral += errorValue * dt;  // Integrate the error over time
//        // Anti-windup: Limit the integral term to prevent it from growing too large
//        integral = Math.max(Math.min(integral, 1000), -1000);
//
//        double derivative = (errorValue - prevError) / dt;
//
//        // Compute the final PID output
//        double output = proportional + ki * integral + kd * derivative;
//
//        // Apply the motor power
//        output = Math.max(Math.min(output, 100), -100);  // Clamp output to motor range
//
//        motors.MoveMotor(Motors.Type.Arm, output);
//
//        // Store current error and time for next iteration
//
//        prevError = errorValue;
//
//        telemetry.addData("Set Point", setPoint);
//        telemetry.addData("Process Value", processValue);
//        telemetry.addData("Proportional Gain", kp);
//        telemetry.addData("Integral Gain", ki);
//        telemetry.addData("Derivative Gain", kd);
//        telemetry.addData("Proportional", proportional);
//        telemetry.addData("Integral", integral);
//        telemetry.addData("Derivative", derivative);
//        telemetry.addData("PID Output", output);
//        telemetry.update();
//
//        dashboardTelemetry.addData("Set Point", setPoint);
//        dashboardTelemetry.addData("Process Value", processValue);
//        dashboardTelemetry.addData("Proportional Gain", kp);
//        dashboardTelemetry.addData("Integral Gain", ki);
//        dashboardTelemetry.addData("Derivative Gain", kd);
//        dashboardTelemetry.addData("Proportional", proportional);
//        dashboardTelemetry.addData("Integral", integral);
//        dashboardTelemetry.addData("Derivative", derivative);
//        dashboardTelemetry.addData("PID Output", output);
//        dashboardTelemetry.update();
//    }
}
