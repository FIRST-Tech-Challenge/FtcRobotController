package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class PixelFingerSubsystem extends SubsystemBase {
    private ServoImplEx servo;
    private final double MIN = 0.98, MAX = 0.65;

    public PixelFingerSubsystem(HardwareMap hm) {
        servo = hm.get(ServoImplEx.class, "pixel_holder");

        grab();
    }

    public void grab() {
        servo.setPosition(MIN);
    }

    public void release() {
        servo.setPosition(MAX);
    }
}
