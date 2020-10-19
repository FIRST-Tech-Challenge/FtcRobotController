package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.virtual.VirtualHardwareManager;
import org.firstinspires.ftc.teamcode.virtual.VirtualMotor;

public abstract class VirtualHardware extends RobotHardware {

    public VirtualHardwareManager vhManager = new VirtualHardwareManager();
    VirtualMotor frontLeft;
    VirtualMotor frontRight;
    VirtualMotor backLeft;
    VirtualMotor backRight;

    @Override
    public void initializeHardware() {
        try {
            vhManager.connect("192.168.1.22", 37564);
            vhManager.setRobotHardware(this);
        } catch (Exception e) {
            telemetry.addData("error initializing virtual hardware", e.getLocalizedMessage());
        }
        //frontLeft = vhManager.initializeVirtualDevice(VirtualMotor.class, "frontLeft");
    }

    @Override
    public void stop() {
        super.stop();
        vhManager.disconnect();
    }
}
