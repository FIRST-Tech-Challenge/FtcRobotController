package org.firstinspires.ftc.teamcode.opmodes.Commands;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public interface Command {
    void Execute();
    SubSystem getHardwareDevice();
}
