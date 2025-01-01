package org.firstinspires.ftc.teamcode.Hardware.Util;

import androidx.lifecycle.MutableLiveData;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger {

    public static enum LogLevels {
        production,
        debug,
        developer;
    }

    private Telemetry telemetry;
    private LogLevels state;
    private GamepadEx controller;

    public Logger(Telemetry telemetry, GamepadEx controller) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        state = LogLevels.production;
        this.controller = controller;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    public void log(String label, Object data, LogLevels level) {
        switch (state) {
            case production:

                if (level == LogLevels.production) {
                    telemetry.addData(label, data);
                }
                break;

            case debug:

                if (level == LogLevels.production || level == LogLevels.debug) {
                    telemetry.addData(label, data);
                }
                break;

            case developer:

                if (level == LogLevels.production || level == LogLevels.debug || level == LogLevels.developer) {
                    telemetry.addData(label, data);
                }
                break;

        }


    }

    public void print() {

        if (controller.wasJustPressed(GamepadKeys.Button.BACK)) {

            if (state == LogLevels.production) {
                state = LogLevels.debug;
            } else if (state == LogLevels.debug) {
                state = LogLevels.developer;
            } else if (state == LogLevels.developer) {
                state = LogLevels.production;
            }

        }

        telemetry.update();
    }



}
