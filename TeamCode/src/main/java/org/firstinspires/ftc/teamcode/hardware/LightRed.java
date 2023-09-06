package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LightRed extends Lights {
    public LightRed(DcMotor lights) {
        super(lights);
    }

    public void on() {
        this.redOn();
    }

}
