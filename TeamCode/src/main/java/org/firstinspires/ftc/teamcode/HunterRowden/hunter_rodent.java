package org.firstinspires.ftc.teamcode.HunterRowden;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class hunter_rodent extends OpMode {
    public int arm;
    @Override
    public void init() {
        int a  = 5;
        int b = 6;
        arm = a+b;

    }

    @Override
    public void loop() {
        telemetry.addLine(String.valueOf(arm));
    }
}
