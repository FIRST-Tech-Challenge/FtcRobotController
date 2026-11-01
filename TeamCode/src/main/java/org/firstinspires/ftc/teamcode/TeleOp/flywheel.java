package org.firstinspires.ftc.teamcode.TeleOp;

import android.icu.text.UFormat;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * TeleOp OpMode for testing and live-tuning FTC flywheel motor velocity and PIDF coefficients.
 * <p>
 * Gamepad 1 Controls:
 * - Y Button: Toggle target velocity between High (1500) and Low (900) ticks/sec.
 * - B Button: Cycle tuning step size (10.0, 1.0, 0.1, 0.01, 0.001, 0.0001).
 * - D-Pad Left / Right: Increase / Decrease Feedforward gain (F).
 * - D-Pad Up / Down: Increase / Decrease Feedforward gain (F).
 */
@TeleOp
public class flywheel extends OpMode {
    /** Motor instance for controlling the flywheel with encoder velocity control. */
    public DcMotorEx flywheelMotor;
    /** Target velocity presets in encoder ticks per second. */
    double highVelocity = 1500;
    double lowVelocity = 900;

    /** Currently active target velocity preset. */
    double curTargetVelocity = highVelocity;

    /** Feedforward (F) and Proportional (P) PIDF gain coefficients. */
    double F = 0;
    double P = 0;

    /** Array of step magnitudes used when adjusting tuning values via gamepad controls. */
    double[] stepsizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    /** Index of the active step size in the stepsizes array. */
    int stepIndex = 1;

    /**
     * Initializes the flywheel motor hardware, configures encoder mode, sets motor direction,
     * and sets initial PIDF coefficients.
     */
    @Override
    public void init() {
        // Retrieve flywheel motor hardware instance
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Enable closed-loop velocity mode using built-in motor encoder
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor direction to reversed
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure initial PIDF coefficients on the motor (P, I=0, D=0, F)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    /**
     * Main loop executed iteratively while the OpMode is running.
     * Handles gamepad inputs for target velocity toggling, step-size selection, live F tuning,
     * motor velocity updating, and real-time telemetry display.
     */
    @Override
    public void loop() {

        // Y Button: Toggle target velocity between highVelocity (1500) and lowVelocity (900)
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity;  }
        }

        // B Button: Cycle through the step sizes array for tuning adjustments
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepsizes.length;
            }

        // D-Pad Left: Increase Feedforward gain (F) by the current step size
        if (gamepad1.dpadLeftWasPressed()) {
            F += stepsizes[stepIndex];
        }

        // D-Pad Right: Decrease Feedforward gain (F) by the current step size
        if (gamepad1.dpadRightWasPressed()) {
            F -= stepsizes[stepIndex];
        }

        // D-Pad Up: Increase Feedforward gain (F) by the current step size
        if (gamepad1.dpadUpWasPressed()) {
            F += stepsizes[stepIndex];
        }

        // D-Pad Down: Decrease Feedforward gain (F) by the current step size
        if (gamepad1.dpadDownWasPressed()) {
            F-= stepsizes[stepIndex];
        }

        // set new PIDF coefficients :)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        // set velocity of thingy
        flywheelMotor.setVelocity(curTargetVelocity);

        // Measure actual motor velocity and calculate error
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;



        // Send target velocity, current velocity, error, and current PIDF parameters to Driver Station telemetry
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepsizes[stepIndex]);

    }
}