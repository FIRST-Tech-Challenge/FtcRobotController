package org.firstinspires.ftc.teamcode.utils.motors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TetrixMotor extends Motor {

    public TetrixMotor(Telemetry telemetry, HardwareMap hardware, String name, int offset) {
        super(telemetry, hardware, name, offset, 1400, 2.0, 2.0);
    }

}
