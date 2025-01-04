package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
//@TeleOp(name = "Arm PID Test")
public class ArmPIDTest extends LinearOpMode {


    /* Calculate Arm positions

    take motor.getArmPosition() when it is resting inside the robot
    and add something like 400 to get arm position at vertical
    and add another number like 800 to get forward arm position

    This will allow the robot to correct for a slip gear or something like that
     */


    Motors motors;
    ElapsedTime elapsedTime;

    // PID variables
     public static double kp = 2;  // Proportional gain
     public static double ki = 0.45;  // Integral gain
     public static double kd = 0.13;  // Derivative gain


     static int setPoint;

    double prevError = 0;  // Previous error, used for derivative
    double integral = 0;   // Integral term

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();  //REMOVE THIS
        Telemetry dashboardTelemetry = dashboard.getTelemetry(); //AND THIS BEFORE COMPETITION also line 109

        motors = new Motors(hardwareMap);
        elapsedTime = new ElapsedTime();

        setPoint = motors.getArmPosition(); // init it as arm position because if it was for example 0, the arm could slam into the ground and break

        waitForStart();

        double prevTime = elapsedTime.milliseconds();

        while (opModeIsActive()) {

            setPoint += (int) (gamepad2.left_stick_y * 100);    //multiply the game pad input by 100 so that there are no decimals which doesn't work in the setPoint then turn it into and int
            // Get current time and arm position
            double time = elapsedTime.milliseconds();

            // Time difference (dt)
            double dt = (time - prevTime) / 1000.0;  // Convert to seconds

            double processValue = motors.getArmPosition(); // Finding the position of the motors

            // Calculate error
            double errorValue = setPoint - processValue;



            // Prevent divide-by-zero errors
            if (dt == 0) {
                dt = 0.1;  // Default small value if no time has passed
            }

            // Calculate PID terms
            double proportional = kp * errorValue;

            integral += errorValue * dt;  // Integrate the error over time
            // Anti-windup: Limit the integral term to prevent it from growing too large
            integral = Math.max(Math.min(integral, 1000), -1000);

            double derivative = (errorValue - prevError) / dt;

            // Compute the final PID output
            double output = proportional + ki * integral + kd * derivative;

            // Apply the motor power
            output = Math.max(Math.min(output, 100), -100);  // Clamp output to motor range

            motors.MoveMotor(Motors.Type.Arm, output);

            // Store current error and time for next iteration

            prevError = errorValue;
            prevTime = time;



            // Telemetry
            telemetry.addData("Set Point", setPoint);
            telemetry.addData("Process Value", processValue);
            telemetry.addData("Proportional Gain", kp);
            telemetry.addData("Integral Gain", ki);
            telemetry.addData("Derivative Gain", kd);
            telemetry.addData("Proportional", proportional);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("PID Output", output);
            telemetry.update();

            dashboardTelemetry.addData("Set Point", setPoint);
            dashboardTelemetry.addData("Process Value", processValue);
            dashboardTelemetry.addData("Proportional Gain", kp);
            dashboardTelemetry.addData("Integral Gain", ki);
            dashboardTelemetry.addData("Derivative Gain", kd);
            dashboardTelemetry.addData("Proportional", proportional);
            dashboardTelemetry.addData("Integral", integral);
            dashboardTelemetry.addData("Derivative", derivative);
            dashboardTelemetry.addData("PID Output", output);
            dashboardTelemetry.update();
        }
    }
}
