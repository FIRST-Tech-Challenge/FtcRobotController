package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FrontSliderSubsystem extends SubsystemBase {
    private final ServoEx servo1, servo2;

    private double targetPos;
    private double MIN_ANGLE = 0.0, MAX_ANGLE = 180.0;

    private double step = 1;

    public enum State {
        OPENING, CLOSING
    }

    private State state;

    public FrontSliderSubsystem(HardwareMap hardwareMap) {
        servo1 = new SimpleServo(hardwareMap, "servo1", MIN_ANGLE, MAX_ANGLE,
                AngleUnit.DEGREES);
        servo2 = new SimpleServo(hardwareMap, "servo2", MIN_ANGLE, MAX_ANGLE,
                AngleUnit.DEGREES);
    }

    public void increasePosition() {
        targetPos += targetPos + step <= MAX_ANGLE ? step : 0;
        setPosition(targetPos);
    }

    public void decreasePosition() {
        targetPos -= targetPos - step >= MAX_ANGLE ? step : 0;
        setPosition(targetPos);
    }

    public void open() {
        setPosition(180);
    }

    public void close() {
        setPosition(0);
    }

    public void stop() {
        //
    }

    private void setPosition(double pos) {
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }

    public void setPower(double power) {
        //
    }

    public State getState() {
        return state;
    }
}
