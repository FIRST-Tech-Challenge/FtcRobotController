package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class SpecimenArmAndClaw extends SonicSubsystemBase {

    private Servo clawServo;

    private Servo armServo;


    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public SpecimenArmAndClaw(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.armServo  = hardwareMap.get(Servo.class,"SpecimenArm");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

    }

    public void MoveToIntakePosition() {
        armServo.setPosition(0.29);
        armServo.setPosition(0.25);

    }

    public void MoveToDeliveryPosition() {
        armServo.setPosition(1);
    }

}