package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Constants;
import org.firstinspires.ftc.teamcode.Systems.Motors;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
@TeleOp(name = "Arm PID Test")
public class ArmPIDTest extends LinearOpMode {


    /* Calculate Arm positions

    take motor.getArmPosition() when it is resting inside the robot
    and add something like 400 to get arm position at vertical
    and add another number like 800 to get forward arm position

    This will allow the robot to correct for a slip gear or something like that
     */


    Motors motors;
    ElapsedTime elapsedTime;


     static int setPoint;

    double prevError = 0;  // Previous error, used for derivative
    double integral = 0;   // Integral term

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry(); //AND THIS BEFORE COMPETITION also line 109
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        motors = new Motors(hardwareMap);
        elapsedTime = new ElapsedTime();

        setPoint = motors.getArmPosition(); // init it as arm position because if it was for example 0, the arm could slam into the ground and break

        waitForStart();

        double prevTime = elapsedTime.milliseconds();

        while (opModeIsActive()) {


            setPoint += (int) (-gamepad2.right_stick_y * 35);    // Multiply the game pad input by a number so that we can tune the sensitivity then turn it into and int so the code can work

            // Clamp setPoint between resting and reaching positions
            setPoint = Math.max(motors.getArmRestingPosition(), Math.min(setPoint, motors.getArmReachingPosition()));

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
            double proportional = Constants.KP * errorValue;

            integral += errorValue * dt;  // Integrate the error over time
            // Anti-windup: Limit the integral term to prevent it from growing too large
            integral = Math.max(Math.min(integral, 1000), -1000);

            double derivative = (errorValue - prevError) / dt;

            // Compute the final PID output
            double output = proportional + Constants.KI * integral + Constants.KD * derivative;

            // Apply the motor power
            output = Math.max(Math.min(output, 50), -50);  // Clamp output to motor range and make it so that it will more slowly go to its target position

            if((motors.getArmPosition() == motors.getArmRestingPosition()) && (gamepad2.right_stick_y == 0)) { // Allows the arm to not be powered when it is in its resting position and no inputs are given
                motors.MoveMotor(Motors.Type.Arm,0);                                // This can prevent the motor from overheating like it was doing earlier
            }
            else {
                motors.MoveMotor(Motors.Type.Arm, output);
            }



            // Store current error and time for next iteration
            prevError = errorValue;
            prevTime = time;


            BotTelemetry.addData("Set Point", setPoint);
            BotTelemetry.addData("Process Value", processValue);
            BotTelemetry.addData("Proportional Gain", Constants.KP);
            BotTelemetry.addData("Integral Gain", Constants.KI);
            BotTelemetry.addData("Derivative Gain", Constants.KD);
            BotTelemetry.addData("Proportional", proportional);
            BotTelemetry.addData("Integral", integral);
            BotTelemetry.addData("Derivative", derivative);
            BotTelemetry.addData("PID Output", output);

            BotTelemetry.update();

        }
    }
}
