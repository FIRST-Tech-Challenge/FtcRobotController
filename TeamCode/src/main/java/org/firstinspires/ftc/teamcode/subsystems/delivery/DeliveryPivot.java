package org.firstinspires.ftc.teamcode.subsystems.delivery;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class DeliveryPivot extends SonicSubsystemBase {

    private Motor motor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public DeliveryPivot(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.motor  = new Motor(hardwareMap, "DeliveryPivot");

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;
    }

    public void RotateTowardsIntake() {
        motor.set(-1);
    }

    public void RotateTowardsDelivery() {
        motor.set(1);
    }

    public void RotateTowardsIntakeSlowly() {
        motor.set(-.33);
    }

    public void RotateTowardsDeliverySlowly() {
        motor.set(.33);
    }

    public void HoldArm() {
        motor.set(0);
    }
}