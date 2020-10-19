package org.firstinspires.ftc.teamcode.virtual;

public abstract class VirtualDevice {

    VirtualHardwareManager parentConnectionManager;
    String deviceName;
    boolean dataReadyToSend;

    public void setParentConnectionManager(VirtualHardwareManager parentConnectionManager) {
        this.parentConnectionManager = parentConnectionManager;
    }

    abstract void updateDevice();
}
