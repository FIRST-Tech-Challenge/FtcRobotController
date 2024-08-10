package com.qualcomm.robotcore.hardware;

public abstract class I2cDeviceSynchDevice<DEVICE_CLIENT extends I2cDeviceSynchSimple> implements HardwareDevice {
    protected DEVICE_CLIENT deviceClient;

    protected I2cDeviceSynchDevice(DEVICE_CLIENT deviceClient, boolean deviceClientIsOwned) {
    }

    protected void registerArmingStateCallback(boolean doInitialCallback) {
    }

    protected abstract boolean doInitialize();

    protected void engage() {
    }

    protected void disengage() {
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override public void close() {
    }

    @Override
    public int getVersion()
    {
        return 1;
    }

    @Override
    public String getConnectionInfo()
    {
        return "";
    }
}
