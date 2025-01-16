package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Claw {
    public final Servo clawServo;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    public boolean opened = true;

    public Claw(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        clawServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.CLAW);
        close();
    }

    public void open() {
        clawServo.setPosition(Settings.Hardware.Servo.Claw.OPEN);
        opened = true;
    }

    /* Close both servos */
    public void close() {
        clawServo.setPosition(Settings.Hardware.Servo.Claw.CLOSED);
        opened = false;
    }

    public void toggle() {
        if (opened) {
            close();
        } else {
            open();
        }
    }


}
