package org.firstinspires.ftc.teamcode.Enhancement.Config;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;

/**
 * Important configuration
 */
public class MainConfig extends Config {
    private static final String name = "UwU the Potato";

    private static final String version = "2021.10.30.0-alpha";
    private static final boolean debug = false;
    /* "none" means no target
    enter a file path to debug a file
    enter a folder path to debug a folder
    "*" means debug all
     */
    private static final String debugTarget = "none";
    /* Scale from 0-5*/
    private static final int logLevel = 3;
    private static final boolean initSubsystems = true;
    private static final boolean initSubsystemControl = (true && initSubsystems);
    private static final boolean initSubsystemDrive = (true && initSubsystems);
    private static final boolean initSubsystemVision = (true && initSubsystems);
    private static final boolean initMechanical = true;
    private static final boolean initGetGamePadInputs = true;
    private static final boolean initHardwareMap = true;
    private static AllianceColor allianceColor = AllianceColor.BLUE;

    public static String getName() {
        return name;
    }

    public static String getVersion() {
        return version;
    }

    public static AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public static void setAllianceColor(AllianceColor color) {
        allianceColor = color;
    }

    public static boolean getDebug() {
        return debug;
    }

    public static String getDebugTarget() {
        return debugTarget;
    }

    public static int getLogLevel() {
        return logLevel;
    }

    public static boolean getInitSubsystems() {
        return initSubsystems;
    }

    public static boolean getInitSubsystemControl() {
        return initSubsystemControl;
    }

    public static boolean getInitSubsystemDrive() {
        return initSubsystemDrive;
    }

    public static boolean getInitSubsystemVision() {
        return initSubsystemVision;
    }

    public static boolean getInitMechanical() {
        return initMechanical;
    }

    public static boolean getInitGamePadInputs() {
        return initGetGamePadInputs;
    }

    public static boolean getInitHardwareMap() {
        return initHardwareMap;
    }

    @Override
    public Object get(String key) {
        return null;
    }

    @Override
    public void set(String key, Object value) {

    }
}
