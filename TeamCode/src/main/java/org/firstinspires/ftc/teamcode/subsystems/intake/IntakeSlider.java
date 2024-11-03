package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class IntakeSlider extends SonicSubsystemBase {

    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public IntakeSlider(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "IntakeSlider");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.motor.encoder.reset();
    }

    public void Expand() {
        motor.set(.5);
    }

    public void Collapse() {
        motor.set(-.5);
    }

    public void Hold() {
        motor.set(0);
    }
}