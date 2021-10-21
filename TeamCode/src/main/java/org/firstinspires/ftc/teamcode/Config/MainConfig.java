package org.firstinspires.ftc.teamcode.Config;

import org.firstinspires.ftc.teamcode.Util.AllianceColor;

/** Important configuration
 *
 */
public class MainConfig {
    private static final String name = "Freight Mover"; // TODO: Better name needed
    private static final String version = "0.0.0-alpha";
    private static final AllianceColor allianceColor = AllianceColor.BLUE;
    private static final boolean debug = false;
    // 0 is quiet
    // 1 is default
    // 2 is rich
    // 3 is verbose
    private static final int logLevel = 1;

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

    public static int getLogLevel() {
        return logLevel;
    }
}
