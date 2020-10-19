package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.VirtualHardware;

@TeleOp(name="Virtual Hardware")
public class VirtualHardwareTest extends VirtualHardware {

    public VirtualHardwareTest() {
        this.initVuforia = false;
    }

    @Override
    public void initializeHardware() {
        super.initializeHardware();
    }

    @Override
    public void loop() {

    }
}
