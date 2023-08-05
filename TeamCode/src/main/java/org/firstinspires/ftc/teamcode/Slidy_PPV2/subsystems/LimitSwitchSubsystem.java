package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchSubsystem extends SubsystemBase {
    DigitalChannel limSwitch;

    public LimitSwitchSubsystem(HardwareMap hm, String name) {
        limSwitch = hm.get(DigitalChannel.class, name);
        limSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState() {
        return limSwitch.getState();
    }
}
