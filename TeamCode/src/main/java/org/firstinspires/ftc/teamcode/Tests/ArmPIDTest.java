package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Arm PID Test")
public class ArmPIDTest extends LinearOpMode {

    Motors motors;
    ElapsedTime elapsedTime;

    // PID variables
    public static double kp = 1;  // Proportional gain
    public static double ki = 1;  // Integral gain
    public static double kd = 0;  // Derivative gain

    double prevError = 0;  // Previous error, used for derivative
    double integral = 0;   // Integral term

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        motors = new Motors(hardwareMap);
        elapsedTime = new ElapsedTime();

        waitForStart();

        double prevTime = elapsedTime.milliseconds();
        double prevArmPos = motors.getArmPosition();

        while (opModeIsActive()) {

            // Get current time and arm position
            double time = elapsedTime.milliseconds();
            double armPos = motors.getArmPosition();

            // Setpoint changes based on joystick input
            double setPoint = gamepad2.right_stick_y * 100;  // Example scaling factor to setpoint

            // Time difference (dt)
            double dt = (time - prevTime) / 1000.0;  // Convert to seconds

            double processValue = ((armPos - prevArmPos) / dt); // Finding the velocity of the motors

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
            output = Math.max(Math.min(output, 50), -50);  // Clamp output to motor range

            motors.MoveMotor(Motors.Type.Arm, output);

            // Store current error and time for next iteration
            prevError = errorValue;
            prevTime = time;
            prevArmPos = armPos;

//            if(gamepad2.dpad_up)
//                kp++;
//            else if (gamepad2.dpad_right)
//                ki++;
//            else if (gamepad2.dpad_down)
//                kd++;




            // Telemetry
            telemetry.addData("Setpoint", setPoint);
            telemetry.addData("Process Value", processValue);
            telemetry.addData("PropGain", kp);
            telemetry.addData("IntegGain", ki);
            telemetry.addData("DerivGain", kd);
            telemetry.addData("Arm Position", armPos);
            telemetry.addData("Proportional", proportional);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("PID Output", output);
            telemetry.update();

            dashboardTelemetry.addData("Set Point", setPoint);
            dashboardTelemetry.addData("Process Value", processValue);
            dashboardTelemetry.addData("PropGain", kp);
            dashboardTelemetry.addData("IntegGain", ki);
            dashboardTelemetry.addData("DerivGain", kd);
            dashboardTelemetry.addData("Arm Position", armPos);
            dashboardTelemetry.addData("Proportional", proportional);
            dashboardTelemetry.addData("Integral", integral);
            dashboardTelemetry.addData("Derivative", derivative);
            dashboardTelemetry.addData("PID Output", output);
            dashboardTelemetry.update();
        }
    }
}
