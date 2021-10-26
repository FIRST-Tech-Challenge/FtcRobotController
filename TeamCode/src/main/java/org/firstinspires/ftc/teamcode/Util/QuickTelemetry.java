package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.MainConfig;


/** Extention of {@link Telemetry} that is easier to use and is more feature rich.
 *
 * @see Telemetry
 */
public class QuickTelemetry {
    private Telemetry telemetry;
    private final int logLevel = MainConfig.getLogLevel();
    private String file = "";
    private String targetDebug = MainConfig.getDebugTarget();

    public QuickTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public QuickTelemetry(Telemetry telemetry, String file) {
        this.telemetry = telemetry;
        this.file = file;
    }

    public boolean canLog(int logLevel) {
        return ((logLevel >= this.logLevel) || (this.file.equals(this.targetDebug)));
    }

    public void update() {
        this.telemetry.update();
    }

    public QuickTelemetry newQuickTelemetryFile(String file) {
        return new QuickTelemetry(this.telemetry, file);
    }

    public void telemetry(String format) {
        if (logLevel > 0) {
            this.telemetry.addData(this.file, format);
            update();
        }
    }

    public void telemetry(String caption, String format) {
        if (logLevel > 0) {
            this.telemetry.addData(this.file + ": " + caption, format);
            update();
        }
    }

    public void telemetry(String caption, String format, Object... args) {
        if (logLevel > 0) {
            this.telemetry.addData(this.file + ": " + caption, format, args);
            update();
        }
    }

    public void telemetry(int logLevel, String caption, String format, Object... args) {
        if (logLevel == 0) {
            throw new RuntimeException("You can't have a logLevel of 0");
        }
        if (canLog(logLevel)) {
            this.telemetry(caption, format, args);
        }
    }

    public void telemetry(int logLevel, String caption, String format) {
        if (logLevel == 0) {
            throw new RuntimeException("You can't have a logLevel of 0");
        }
        if (canLog(logLevel)) {
            this.telemetry(caption, format);
        }
    }

    public void telemetry(int logLevel, String format) {
        if (logLevel == 0) {
            throw new RuntimeException("You can't have a logLevel of 0");
        }
        if (canLog(logLevel)) {
            this.telemetry(format);
        }
    }
}
