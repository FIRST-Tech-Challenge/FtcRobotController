package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TeamHardwareMap {
    public ElapsedTime Runtime = new ElapsedTime();

    protected HardwareMap hardwareMap;

    public TeamHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialise();
    }

    public abstract void initialise();
}
