package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger implements Log {
    private static final Logger instance = new Logger();
    private Telemetry telemetry = null;

    private Logger() {
    }

    public static Logger getInstance() {
        return instance;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void log(CAPTION caption, String format, Object... args) {
        if (null == telemetry) {
            System.out.println(String.format(format, args));
        } else {
            telemetry.addData(caption.name(), format, args);
            telemetry.update();
        }
    }

    @Override
    public void log(CAPTION caption, Object message) {
        if (null == telemetry) {
            System.out.println(message);
        } else {
            telemetry.addData(caption.name(), message);
            telemetry.update();
        }
    }

}
