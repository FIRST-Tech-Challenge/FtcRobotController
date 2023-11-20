package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashMap;

public class VirtualDevice implements HardwareDevice {

    public Manufacturer manufacturer;
    public String name;
    public String partID;
    protected HashMap<String, Integer> connectionScheme;
    public int version;

    public VirtualDevice(String partNumber, String DeviceName, HashMap<String, Integer> connectionScheme, int version) {
        this.name = DeviceName;
        this.partID = partNumber;
        this.version = version;
        this.connectionScheme = connectionScheme;
    }
    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
    @Override
    public String getDeviceName() {
        return name;
    }
    public String getPartName() {
        return partID;
    }
    @Override
    public String getConnectionInfo() {
        return connectionScheme.toString();
    }
    @Override
    public int getVersion() {
        return version;
    }
    @Override
    public void resetDeviceConfigurationForOpMode() {

    }
    @Override
    public void close() {

    }
}
