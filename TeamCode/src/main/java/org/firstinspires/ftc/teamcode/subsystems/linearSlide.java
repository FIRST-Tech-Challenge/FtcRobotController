package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class linearSlide {

    private DcMotor slideMotor;

    private static final double RobotBase_to_LS = 219.160;  // Distance from the front of the robot to the slide base
    private static final double Max_Extension = 42; // Max safe extension distance from slide base

    // PID constants
    private double kP = 0.01;
    private double kI = 0.0001;
    private double kD = 0.001;

    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;  // Starting at initial position (adjust)
    double power;
    OpMode opMode;
    public linearSlide(OpMode opMode) {
        opMode = opMode;
    }
    public void init() {
        slideMotor = opMode.hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if (opMode.gamepad1.dpad_up) {
            manualUp();
        } else if (opMode.gamepad1.dpad_down) {
            manualDown();
        }
        targetPosition = Math.max(0, Math.min(targetPosition, 360));

        // Calculate current forward reach
        double currentExtension = slideMotor.getCurrentPosition() + RobotBase_to_LS;

        // Ensure the slide does not exceed max forward extension
        if ((targetPosition > slideMotor.getCurrentPosition() && currentExtension < Max_Extension) ||
                (targetPosition < slideMotor.getCurrentPosition() && currentExtension > RobotBase_to_LS)) {

            double power = calculatePID(targetPosition, slideMotor.getCurrentPosition());
            slideMotor.setPower(power);
        } else {
            slideMotor.setPower(0);  // Stop motor if exceeding extension limits (failsafe)
        }

        // Update the slide motor power using PID
        power = calculatePID(targetPosition, slideMotor.getCurrentPosition());
        slideMotor.setPower(power);
    }
        // Initialize motor
    public void manualUp() {
        targetPosition += 0;
    }

    public void manualDown() {
        targetPosition -= 0;
    }



            // Clamp target position to a safe range (adjust limits as needed)
    public void getTelemetry() {
        // Telemetry for debugging
        opMode.telemetry.addData("Target Position", targetPosition);
        opMode.telemetry.addData("Current Position", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Power", power);
        opMode.telemetry.update();
    }
    public void getTelemetry(Telemetry t) {
        // Telemetry for debugging
        t.addData("Target Position", targetPosition);
        t.addData("Current Position", slideMotor.getCurrentPosition());
        t.addData("Power", power);
        t.update();
    }

    private double calculatePID(int target, int current) {
        // Calculate error
        double error = target - current;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        integral += error;
        double iTerm = kI * integral;

        // Derivative term
        double dTerm = kD * (error - lastError);
        lastError = error;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
