package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchSubsystem extends SubsystemBase {
    final DigitalChannel limitSwitch;

    public LimitSwitchSubsystem(HardwareMap hm, String name) {
        limitSwitch = hm.get(DigitalChannel.class, name);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState() {
        return limitSwitch.getState();
    }
}

