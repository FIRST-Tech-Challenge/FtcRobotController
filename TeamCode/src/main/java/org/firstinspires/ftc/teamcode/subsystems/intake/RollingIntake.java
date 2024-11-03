package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class RollingIntake extends SonicSubsystemBase {

    private CRServo leftServo;

    private CRServo rightServo;

    private Servo elbowServo;


    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    public RollingIntake(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.rightServo  = hardwareMap.get(CRServo.class,"RightIntake");
        this.leftServo  = hardwareMap.get(CRServo.class,"LeftIntake");

        this.elbowServo = hardwareMap.get(Servo.class, "Elbow");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.elbowServo.setPosition(0.5);
    }
    public void SetElbowInIntakePosition() {
        this.elbowServo.setPosition(0.5);
    }
    public void SetElbowInSpecimenPosition() {
        this.elbowServo.setPosition(0.85);

    }

    public void Intake() {
        this.leftServo.setPower(1);
        this.rightServo.setPower(-1);
    }

    public void Outtake() {
        this.leftServo.setPower(-1);
        this.rightServo.setPower(1);
    }

    public void Hold(){
        this.leftServo.setPower(0);
        this.rightServo.setPower(0);
    }
}