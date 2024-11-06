package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;

/** @noinspection unused */
public class Settings {
    // makes strafing faster or slower
    public static final double strafe_power_coefficient = 1.2;
    public static final double tileLengthFeet = 2;
    public static double ms_needed_to_park = 10000;


    public static class DefaultGamepadSettings {
        public double left_stick_sensitivity = 1.0;
        public double dpad_sensitivity = 0.3;
        public double bumper_sensitivity = 0.8;
        public double trigger_threshold = 0.1;
    }

    public static String getDisabledFlags() {
        StringBuilder enabledFlags = new StringBuilder();

        Field[] fields = Deploy.class.getFields();

        for (Field field : fields) {
            try {
                if (!field.getBoolean(null)) {
                    enabledFlags.append(field.getName()).append(", ");
                }
            } catch (IllegalAccessException ignored) {}
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

        // magic
        public static final boolean ODOMETRY = true;
        public static final boolean DEBUG = true;
        public static final boolean VICTORY = false;

        public static final boolean SKIP_AUTONOMOUS = false;

    }

}
