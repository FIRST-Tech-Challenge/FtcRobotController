package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class GeckoWheels {
    private final CRServo geckoLeft;
    private final CRServo geckoRight;
    private final BaseRobot baseRobot;

    public GeckoWheels(@NonNull BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.geckoLeft = baseRobot.hardwareMap.get(CRServo.class, Settings.Hardware.IDs.GECKO_LEFT);
        this.geckoRight = baseRobot.hardwareMap.get(CRServo.class, Settings.Hardware.IDs.GECKO_RIGHT);

        geckoRight.setDirection(CRServo.Direction.REVERSE); // so they both spin the same way

        stop();
    }

    public void intake() {
        geckoLeft.setPower(Settings.Hardware.Intake.SPEED);
        geckoRight.setPower(Settings.Hardware.Intake.SPEED);
    }

    public void outtake() {
        geckoLeft.setPower(-Settings.Hardware.Intake.SPEED);
        geckoRight.setPower(-Settings.Hardware.Intake.SPEED);
    }

    public void stop() {
        geckoLeft.setPower(0);
        geckoRight.setPower(0);
    }
}
