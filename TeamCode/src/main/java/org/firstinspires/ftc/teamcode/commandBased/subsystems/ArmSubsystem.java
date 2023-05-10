package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmSubsystem extends SubsystemBase {

    private ServoEx lServo, rServo;

//    public ArmSubsystem(final HardwareMap hwMap) {
//        lServo = hwMap.get(Servo.class, "lArm");
//        rServo = hwMap.get(Servo.class, "rArm");
//
//        lServo.setDirection(Servo.Direction.FORWARD);
//        rServo.setDirection(Servo.Direction.REVERSE);
//        lServo.setPosition(0.5);
//
//        rServo.setPosition(0.5);
//    }

    public ArmSubsystem(final HardwareMap hwMap) {
        lServo = new SimpleServo(hwMap, "lArm", -90, 90, AngleUnit.DEGREES);
        rServo = new SimpleServo(hwMap, "rArm", 30, 210, AngleUnit.DEGREES);

        lServo.setInverted(false);
        rServo.setInverted(true);
    }

    public void rotate(double amount) {
        lServo.rotateByAngle(amount, AngleUnit.DEGREES);
        rServo.rotateByAngle(amount, AngleUnit.DEGREES);
    }

    public void setAngle(double angle) {
        lServo.turnToAngle(angle, AngleUnit.DEGREES);
        rServo.turnToAngle(angle, AngleUnit.DEGREES);
    }
}
