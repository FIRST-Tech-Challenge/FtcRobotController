package org.firstinspires.ftc.teamcode.Config;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;

/** Important configuration
 *
 */
public class MainConfig {
    private static final String name = "UwU the Potato";

    private static final String version = "2021.10.30.0-alpha";
    private static final AllianceColor allianceColor = AllianceColor.BLUE;
    private static final boolean debug = false;
    /* "none" means no target
    enter a file path to debug a file
    enter a folder path to debug a folder
    "*" means debug all
     */
    private static final String debugTarget = "none";
    /* 0 is quiet
       1 is default
       2 is rich
       3 is verbose */
    private static final int logLevel = 3;


    private static final boolean initSubsystems = true;
    private static final boolean initSubsystemControl = (true && initSubsystems);
    private static final boolean initSubsystemDrive = (true && initSubsystems);
    private static final boolean initSubsystemVision = (false && initSubsystems);

    private static final boolean initMechanical = true;
    private static final boolean initGetGamePadInputs = true;
    private static final boolean initHardwareMap = true;

    public static String getName() {
        return name;
    }

    public static String getVersion() {
        return version;
    }

    public static AllianceColor getAllianceColor() {
        return allianceColor;
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
}
