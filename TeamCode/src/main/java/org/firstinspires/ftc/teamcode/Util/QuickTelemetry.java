package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.MainConfig;

public class QuickTelemetry {
    private Telemetry telemetry;
    private final int logLevel = MainConfig.getLogLevel();

    public QuickTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        this.telemetry.update();
    }

    public void telemetry(String caption, String format) {
        if (logLevel > 0) {
            this.telemetry.addData(caption, format);
            update();
        }
    }

    public void telemetry(String caption, String format, Object... args) {
        if (logLevel > 0) {
            this.telemetry.addData(caption, format, args);
            update();
        }
    }

    public void telemetry(int logLevel, String caption, String format, Object... args) {
        if (logLevel == 0) {
            throw new RuntimeException("You can't have a logLevel of 0");
        }
        if (logLevel >= this.logLevel) {
            this.telemetry.addData(caption, format, args);
        }
    }

    public void telemetry(int logLevel, String caption, String format) {
        if (logLevel == 0) {
            throw new RuntimeException("You can't have a logLevel of 0");
        }
        if (logLevel>=this.logLevel) {
            this.telemetry.addData(caption, format);
        }
    }

    /** @deprecated please use {@link #telemetry(String caption, String format)} instead.
     * Here for backwards compatibility.
     */
    @Deprecated
    public void addData(String caption, String format) {
        telemetry(caption, format);
    }

    /**@deprecated please use {@link #telemetry(String caption, String format, Object... args)} instead.
     * Here for backwards compatibility.
     */
    @Deprecated
    public void addData(String caption, String format, Object... args) {
        telemetry(caption, format, args);
    }
}
