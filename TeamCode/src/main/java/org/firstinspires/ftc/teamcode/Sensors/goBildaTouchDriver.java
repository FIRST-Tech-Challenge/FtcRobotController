package org.firstinspires.ftc.teamcode.hardware.Sensors;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class goBildaTouchDriver {
    private DigitalChannel s;

    public goBildaTouchDriver(DigitalChannel s) {
        this.s = s;
    }

    public boolean check() {
        boolean e = this.s.getState();
        return e;
    }
}
