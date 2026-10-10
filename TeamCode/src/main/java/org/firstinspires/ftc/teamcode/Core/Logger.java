package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Logger {
    public static enum LogLevels {
        DEBUG,
        PRODUCTION,

    }

    private Telemetry telemetry;
    private LogLevels state;


    public Logger(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        state = LogLevels.DEBUG;
    }

    public void log(String caption, Object data, LogLevels logLevel) {
        switch (state) {
            case DEBUG:
                if (!(logLevel == LogLevels.PRODUCTION)) {
                    telemetry.addData(caption, data);

                }
                break;
            case PRODUCTION:
                if (logLevel == LogLevels.PRODUCTION) {
                    telemetry.addData(caption, data);
                }
                break;
        }
    }

    public void updateLoggingLevel(boolean D_Pad_Right) {
        if (D_Pad_Right) {
            if (state == LogLevels.PRODUCTION) {
                state = LogLevels.DEBUG;
                telemetry.addData("CURRENT LOGGER STATE", state);
            } else if (state == LogLevels.DEBUG) {
                telemetry.addData("CURRENT LOGGER STATE", state);
                state = LogLevels.PRODUCTION;
            }
        }
    }


    public void print() {
        telemetry.update();
    }

    public boolean DEBUG() {
        return state == LogLevels.DEBUG;
    }

    public boolean PRODUCTION() {
        return state == LogLevels.DEBUG;
    }
}