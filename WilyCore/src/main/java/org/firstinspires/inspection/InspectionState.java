package org.firstinspires.inspection;

import com.wilyworks.simulator.WilyCore;

/**
 * {@link InspectionState} contains the inspection state of either a RC or a DS
 */
@SuppressWarnings("WeakerAccess")
public class InspectionState
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String robotControllerPackage = "com.qualcomm.ftcrobotcontroller";
    public static final String driverStationPackage = "com.qualcomm.ftcdriverstation";

    public static final String NO_VERSION = "";
    public static final int NO_VERSION_CODE = 0;

    public String manufacturer = "";
    public String model = "";
    public String deviceCodename = ""; // For scoring system to auto-select
    public String osVersion = ""; // Android version (e.g. 7.1.1)
    public String controlHubOsVersion = ""; // Control Hub OS version (e.g. 1.1.1)
    public String driverHubOsVersion = "";
    public int controlHubOsVersionNum;
    public int driverHubOsVersionNum;
    public String firmwareVersion = "";
    public int sdkInt;
    public boolean airplaneModeOn;
    public boolean bluetoothOn;
    public boolean wifiEnabled;
    public boolean wifiConnected;
    public boolean wifiDirectEnabled;
    public boolean wifiDirectConnected;
    public int wifiChannel;
    public boolean locationEnabled;
    public String deviceName;
    public double batteryFraction;
    public boolean robotControllerInstalled;
    public boolean driverStationInstalled;
    public int majorSdkVersion;
    public int minorSdkVersion;
    public int pointSdkVersion;
    public String appBuildTime = "";
    public long rxDataCount;
    public long txDataCount;
    public long bytesPerSecond;
    public boolean isDefaultPassword;

    //----------------------------------------------------------------------------------------------
    // Construction and initialization
    //----------------------------------------------------------------------------------------------

    public InspectionState()
    {
        // For deserialization, initialize CH OS version to NO_VERSION value.
        // Otherwise, it will be null when the RC is running 5.x
        controlHubOsVersion = NO_VERSION;

        // Return the device name that the user has configured for the simulator:
        deviceName = WilyCore.config.deviceName;
    }

    public void initializeLocal()
    {
    }
}
