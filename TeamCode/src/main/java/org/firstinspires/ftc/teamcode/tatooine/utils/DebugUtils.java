    package org.firstinspires.ftc.teamcode.tatooine.utils;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    public class DebugUtils {

        /**
         * Logs a debug message to the telemetry if debug mode is enabled.
         *
         * @param telemetry The Telemetry instance to log to.
         * @param debugMode Whether debug mode is enabled.
         * @param subsystem The name of the subsystem logging the message.
         * @param label The label or key for the telemetry message.
         * @param value The value or data to be logged.
         */
        public static void logDebug(Telemetry telemetry, boolean debugMode, String subsystem, String label, Object value) {
            if (debugMode && telemetry != null) {
                telemetry.addData("[" + subsystem + "] " + label, value);
            }
        }

        /**
         * Logs multiple debug messages to the telemetry if debug mode is enabled.
         *
         * @param telemetry The Telemetry instance to log to.
         * @param debugMode Whether debug mode is enabled.
         * @param subsystem The name of the subsystem logging the message.
         * @param messages An array of messages, where each entry is a pair of label and value.
         */
        public static void logDebug(Telemetry telemetry, boolean debugMode, String subsystem, Object[][] messages) {
            if (debugMode && telemetry != null) {
                for (Object[] message : messages) {
                    if (message.length == 2) {
                        String label = String.valueOf(message[0]);
                        Object value = message[1];
                        telemetry.addData("[" + subsystem + "] " + label, value);
                    }
                }

            }
        }

        /**
         * Logs a simple message if debug mode is enabled.
         *
         * @param telemetry The Telemetry instance to log to.
         * @param debugMode Whether debug mode is enabled.
         * @param subsystem The name of the subsystem logging the message.
         * @param message The message to log.
         */
        public static void logDebugMessage(Telemetry telemetry, boolean debugMode, String subsystem, String message) {
            if (debugMode && telemetry != null) {
                telemetry.addLine("[" + subsystem + "] " + message);
            }
        }
    }
