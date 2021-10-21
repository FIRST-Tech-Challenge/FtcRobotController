package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class QuickTelemetry {
    Telemetry telemetry;
    public QuickTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        this.telemetry.update();
    }

    public void telemetry(String caption, String format) {
        this.telemetry.addData(caption, format);
        update();
    }

    public void telemetry(String caption, String format, Object... args) {
        this.telemetry.addData(caption, format, args);
    }

    /** Here for backwards compatibility
     *
     * @deprecated Please use {@link #telemetry(String caption, String format)} instead
     */
    @Deprecated
    public void addData(String caption, String format) {
        telemetry(caption, format);
    }

    /** Here for backwards compatibility
     *
     * @deprecated Please use {@link #telemetry(String caption, String format, Object... args)} instead
     */
    @Deprecated
    public void addData(String caption, String format, Object... args) {
        telemetry(caption, format, args);
    }
}
