package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase
{
    private static final double WRIST_DEPOSIT = 0.65;
    private static final double WRIST1_DEPOSIT = 0.35;
    private static final double WRIST_INTAKE = 0;
    private static final double WRIST1_INTAKE = 1;
    private static final double WRIST_STOW = 0.65;
    private static final double WRIST1_STOW = 0.35;

    public static final double DETECTION_DISTANCE = 40;

    private static Servo wristServo;
    private static Servo wrist1Servo;
    private static DistanceSensor leftDist;
    private static DistanceSensor rightDist;

    public WristSubsystem(final HardwareMap hMap) {
        wristServo = hMap.get(Servo.class, "wrist");
        wrist1Servo = hMap.get(Servo.class, "wrist1");
    }

    public void setWristIntake() {
        wristServo.setPosition(WRIST_INTAKE);
        wrist1Servo.setPosition(WRIST1_INTAKE);
    }

    public void setWristDeposit() {
        wristServo.setPosition(WRIST_DEPOSIT);
        wrist1Servo.setPosition(WRIST1_DEPOSIT);
    }

    public void setWristStow() {
        wristServo.setPosition(WRIST_STOW);
        wrist1Servo.setPosition(WRIST1_STOW);
    }
}
