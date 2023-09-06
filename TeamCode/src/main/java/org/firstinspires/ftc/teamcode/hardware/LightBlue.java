package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LightBlue extends Lights {
    public LightBlue(DcMotor lights) {
        super(lights);
    }

    public void on() {
        this.blueOn();
    }

}
