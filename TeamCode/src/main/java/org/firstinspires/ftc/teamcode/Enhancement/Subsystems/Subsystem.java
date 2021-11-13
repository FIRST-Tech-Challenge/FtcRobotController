package org.firstinspires.ftc.teamcode.Enhancement.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

/**
 * Superclass to all subsystems, it does some bootstrapping for them (Vision, Control, and Drive)
 */
public abstract class Subsystem {
    // protected because of inheritance
    protected QuickTelemetry telemetry;
    protected ElapsedTime timer;
    protected HardwareMap hardwareMap;

    public Subsystem(QuickTelemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.timer = timer;
    }

    public void init() {
    }
}
