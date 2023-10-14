package org.firstinspires.ftc.teamcode.team10515;


/**
 * Provide utility methods for debug logging
 */
public class DbgLog {
    private DbgLog() {}

    /**
     * Tag used by logcat
     */
    public static final String TAG = "FIRST";

    public static final String ERROR_PREPEND = "### ERROR: ";

    /**
     * Log a debug message
     * @param message
     */
    public static void msg(String message) {
            android.util.Log.i(TAG, message);
    }

    public static void msg(String format, Object... args) {
        msg(String.format(format, args));
    }


    /**
     * Log an error message
     * <p>
     * Messages will be prepended with the ERROR_PREPEND string
     * @param message
     */
    public static void error(String message) {
        android.util.Log.e(TAG, ERROR_PREPEND + message);
    }

    public static void error(String format, Object... args) {
        error(String.format(format, args));
    }

    public static void logStacktrace(Exception e) {
        msg(e.toString());
        for (StackTraceElement el : e.getStackTrace()) {
            msg(el.toString());
        }
    }
}