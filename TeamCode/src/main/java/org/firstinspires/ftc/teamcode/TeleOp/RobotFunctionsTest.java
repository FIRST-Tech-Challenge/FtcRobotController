package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Input;
import org.firstinspires.ftc.teamcode.Systems.Motors;

@TeleOp(name="Functions-Test")

@Config
public class RobotFunctionsTest extends LinearOpMode {

    Motors motors;
    Input input;
    ElapsedTime elapsedTime;
    Camera camera;

    // PID variables
    public static double kp = 2;  // Proportional gain
    public static double ki = 0.45;  // Integral gain
    public static double kd = 0;  // Derivative gain


    public static int setPoint;

    double prevError = 0;  // Previous error, used for derivative
    double integral = 0;   // Integral term



    @Override
    public void runOpMode() throws InterruptedException {

        motors = new Motors(hardwareMap);
        input = new Input(hardwareMap);
        elapsedTime = new ElapsedTime();
        camera = new Camera();

        int ARM_RESTING = motors.getArmPosition();
        int ARM_REACH = motors.getArmPosition() + 1000; // plus some number idk what it actually is

        setPoint = ARM_RESTING;
        waitForStart();
        double prevTime = elapsedTime.milliseconds();

        while (opModeIsActive())
        {
            double armPos = motors.getArmPosition(); // comment out this line when actual probably

            double move = gamepad1.left_stick_y * 100;
            double spin = gamepad1.right_stick_x * 100;
            double strafe = gamepad1.left_stick_x * 100;
            double intake = gamepad2.left_stick_y * 100;

            double armRaise = gamepad2.right_stick_y * 100;

            input.Move(move);
            input.Spin(spin);
            input.Strafe(strafe);

            camera.aprilDistance(gamepad2.a, 20, 0.5);


            setPoint += (int) (gamepad2.left_stick_y * 10);    // Multiply the game pad input by a number so that we can tune the sensitivity then turn it into and int so the code can work

            // Clamp setPoint between resting and reaching positions
            setPoint = Math.max(ARM_RESTING, Math.min(setPoint, ARM_REACH));

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
            output = Math.max(Math.min(output, 50), -50);  // Clamp output to motor range and make it so that it will more slowly go to its target position

            motors.MoveMotor(Motors.Type.Arm, output);

            if((motors.getArmPosition() == ARM_RESTING) && (gamepad2.left_stick_y <= 0)) { // Allows the arm to not be powered when it is in its resting position and no inputs are given
                motors.MoveMotor(Motors.Type.Arm,0);                                // This can prevent the motor from overheating like it was doing earlier
            }

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

            telemetry.addData("MOVE:", "left_y (%.2f),", move);
            telemetry.addData("SPIN:", "right_x (%.2f),", spin);
            telemetry.addData("STRAFE:", "left_x (%.2f),", strafe);
            telemetry.addData("ARM:", "arm_Power (%.2f),", intake);
            telemetry.addData("ARM position:", "arm_pos (%.2f),", armPos);
            telemetry.update(); // telemtryy
        }
    }


}
