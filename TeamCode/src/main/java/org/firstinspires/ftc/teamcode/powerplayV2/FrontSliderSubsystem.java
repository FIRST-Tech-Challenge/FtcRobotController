package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FrontSliderSubsystem extends SubsystemBase {
    private final CRServoImplEx servo1, servo2;

    private double MAXPOWER = 1;

    public enum State {
        OPENED, CLOSED
    }

    private State state;

    public FrontSliderSubsystem(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServoImplEx.class, "frontSlR");
        servo2 = hardwareMap.get(CRServoImplEx.class, "frontSlL");
    }


    public void open() {
        servo1.setPower(MAXPOWER);
        servo2.setPower(-MAXPOWER);
    }

    public void close() {
        servo1.setPower(-MAXPOWER);
        servo2.setPower(MAXPOWER);
    }

    public void stop() {
        servo1.setPower(0);
        servo2.setPower(0);
    }

    public void stopRight() {
        servo1.setPower(0);
    }

    public void stopLeft() {
        servo2.setPower(0);
    }

    public void setMaxPower(double pwr) {
        MAXPOWER = pwr;
    }

    public State getState() {
        return state;
    }
}
