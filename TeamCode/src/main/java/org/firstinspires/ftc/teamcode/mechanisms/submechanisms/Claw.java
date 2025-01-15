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
    public boolean opened = false;
    public double closePos = Settings.Hardware.Servo.Claw.CLOSED;
    public double openPos = Settings.Hardware.Servo.Claw.OPEN;

    public Claw(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        clawServo = hardwareMap.get(Servo.class, Settings.Hardware.IDs.CLAW);
        stop();
    }

    public void forward() {
        clawServo.setPosition(openPos);
    }

    /* Close both servos */
    public void backward() {
        clawServo.setPosition(closePos);
    }

    public void stop() {
        clawServo.setPosition(openPos);
    }

}
