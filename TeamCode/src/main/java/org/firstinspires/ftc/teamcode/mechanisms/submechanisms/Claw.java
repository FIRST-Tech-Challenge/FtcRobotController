package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

/** @noinspection FieldCanBeLocal, unused */
public class Claw {
    public final CRServo clawServo;
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    public boolean opened = true;

    public Claw(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        clawServo = hardwareMap.get(CRServo.class, Settings.Hardware.IDs.CLAW);
        stop();
    }

    public void forward() {
        clawServo.setPower(1);
    }

    /* Close both servos */
    public void backward() {
        clawServo.setPower(-1);
    }

    public void stop() {
        clawServo.setPower(0);
    }

}
