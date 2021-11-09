package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Mechanism {
    void init(HardwareMap hardwareMap, boolean red);
    void run(Gamepad gamepad);
}
