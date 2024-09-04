package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;

public class Settings {

    public static final double dpad_sensitivity = 0.3;
    public static final double bumper_sensitivity = 0.8;
    // makes strafing faster or slower
    public static final double strafe_power_coefficient = 1.2;
    public static final double tileLengthFeet = 2;
    // control preferences
    public static final int playerCount = 2;
    public static final double left_stick_sensitivity = 1.0;
    public static final double lateral_offset = (double) 11 / 12;
    public static final double forward_offset = (double) 9 / 12;


    public static String getDisabledFlags() {
        StringBuilder enabledFlags = new StringBuilder();

        Field[] fields = Deploy.class.getFields();

        for (Field field : fields) {
            try {
                if (!field.getBoolean(null)) {
                    enabledFlags.append(field.getName()).append(", ");
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        return enabledFlags.toString();
    }

    public static class Deploy {
        /* Define deployment flags here;
        An in-progress build with experiment features won't crash
        and problems will be able to be isolated if you just
        uncheck dysfunctional deployment flags first.

        Working on a new or experimental feature?
        Flag it so that we can disable it easily!
         */

        // mechanisms
        public static final boolean ARM = true;
        public static final boolean DRONE = true;
        public static final boolean COLOR = true;

        // magic
        public static final boolean ODOMETRY = true;
        public static final boolean DEBUG = true;
        public static final boolean VICTORY = false;

    }

}
