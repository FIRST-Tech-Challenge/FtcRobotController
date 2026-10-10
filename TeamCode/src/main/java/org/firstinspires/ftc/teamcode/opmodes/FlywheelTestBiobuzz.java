package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.time.Duration;
import java.time.Instant;

@TeleOp(name = "Biobuzz Flywheel Test")
public class FlywheelTestBiobuzz extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor bottomCollection;
    private DcMotor topCollection;
    // Track the desired velocity of the flywheel; this allows incrementing/decrementing
    // the velocity and also measuring the difference between actual and desired velocity
    private double desiredFlywheelVelocity;
    // Used to avoid incrementing/decrementing too quickly when multiple up/down keypresses
    // are read within a loop
    private Instant lastVelocityChange;
    private final Duration debounceTime = Duration.ofMillis(500);

    /**
     * Initialize our subsystems; this should be called once at robot startup
     */
    private void initializeSubsystems() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        topCollection = hardwareMap.get(DcMotor.class, "topCollection");
        bottomCollection = hardwareMap.get(DcMotor.class, "bottomCollection");
        desiredFlywheelVelocity = 0;
    }

    /**
     * Process gamepad input for the collection subsystem. This method takes in gamepad input
     * and adjusts the top and bottom collection subsystems accordingly
     */
    private void processGamepadInputForCollectionSubsystem() {
        if (gamepad1.x) {
            bottomCollection.setPower(-1);
            topCollection.setPower(-1);
        } else if (gamepad1.left_bumper) {
            bottomCollection.setPower(1);
            topCollection.setPower(.8);
        } else {
            bottomCollection.setPower(0);
            topCollection.setPower(0);
        }
    }

    /**
     * Check if we're outside the debounce window for flywheel increment/decrement
     * operations. This method exists to avoid reading a single up/down button press
     * as multiple events; reading one button press multiple times causes too much
     * increase/decrease in velocity
     *
     * @return True if a flywheel change is allowed, false otherwise
     */
    private boolean flywheelChangeAllowed() {
        return lastVelocityChange == null ||
                Duration.between(lastVelocityChange, Instant.now()).compareTo(debounceTime) >= 0;
    }

    /**
     * Adjust the desired velocity from its current amount by the specified amount.
     * This is useful to increment or decrement the velocity relative to its current position
     *
     * @param amount The amount to increment or decrement the velocity
     */
    private void adjustDesiredVelocity(double amount) {
        desiredFlywheelVelocity += amount;
        lastVelocityChange = Instant.now();
    }

    /**
     * Process gamepad input for the flywheel subsystem. This method takes in gamepad input
     * and adjusts the flywheel subsystem accordingly. It also records desired
     * and actual flywheel telemetry for observability's sake
     * ===============
     * Controls:
     * right bumper: Reset flywheel to the baseline velocity; use this to turn on flywheel from a stop
     * b button: Stop the flywheel
     * dpad up: Increase the speed of the flywheel
     * dpad down: Decrease the speed of the flywheel
     */
    private void processGamepadInputForFlywheel() {
        if (gamepad1.right_bumper) {
            // Baseline at -1500 ticks; a negative baseline is used based on the direction
            // we want the flywheel to spin
            desiredFlywheelVelocity = -1500;
        } else if (gamepad1.b) {
            desiredFlywheelVelocity = 0;
        } else if (gamepad1.dpad_up && flywheelChangeAllowed()) {
            // Note: Negative velocities actually are spinning the flywheel faster since we
            // start at a negative baseline
            adjustDesiredVelocity(-50);
        } else if (gamepad1.dpad_down && flywheelChangeAllowed()) {
            // Note: Positive velocities actually are spinning the flywheel slower since we
            // start at a negative baseline
            adjustDesiredVelocity(50);
        }
        flywheel.setVelocity(desiredFlywheelVelocity);

        // Record telemetry for the current Flywheel velocity and the desired velocity
        telemetry.addData("flywheelVelocity", flywheel.getVelocity());
        telemetry.addData("desiredFlywheelVelocity", desiredFlywheelVelocity);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeSubsystems();

        waitForStart();

        while (opModeIsActive()) {
            processGamepadInputForCollectionSubsystem();
            processGamepadInputForFlywheel();
        }
    }
}
