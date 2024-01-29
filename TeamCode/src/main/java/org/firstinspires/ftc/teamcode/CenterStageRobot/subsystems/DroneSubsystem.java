package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DroneSubsystem extends SubsystemBase {
    private ServoImplEx servo;

    public DroneSubsystem(HardwareMap hm) {
        servo = hm.get(ServoImplEx.class, "drone");
    }

    public void grab() {
        servo.setPosition(0);
    }

    public void release() {
        servo.setPosition(0.5);
    }
}
