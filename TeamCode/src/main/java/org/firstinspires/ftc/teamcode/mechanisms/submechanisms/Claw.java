package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Claw {
    public final Servo clawServo;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    public boolean opened = true;

    public Claw(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        clawServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.CLAW);
        clawServo.setDirection(Servo.Direction.REVERSE);
        close();
    }

    /* Set the right servo; true = open, false = close */
    public void setServo(boolean open) {
        double position = open ? Settings.Hardware.Servo.Claw.OPEN : Settings.Hardware.Servo.Claw.CLOSED;
        clawServo.setPosition(position);
    }

    /* Open both servos */
    public void open() {
        if (!opened) {
            setServo(true);
            opened = true;
        }
    }

    /* Close both servos */
    public void close() {
        if (opened) {
            setServo(false);
            opened = false;
        }
    }

}
