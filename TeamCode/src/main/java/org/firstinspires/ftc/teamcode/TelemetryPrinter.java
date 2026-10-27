package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Arrays;

public class TelemetryPrinter {
    private OpMode opMode = null;

    public TelemetryPrinter(OpMode opMode1) {opMode = opMode1;}

    public void print(String input){
        opMode.telemetry.addData(">", input);
    }

    private void _log(String level, String message) {
        this.print(String.format("%s: %s", level.toUpperCase(), message));
    }

    public void helloWorld(String name) {
        this.print(String.format("Hello, %s", name));
    }

    public void log(String message) {
        this._log("log", message);
    }

    public void debug(String message) {
        this._log("debug", message);
    }

    public void error(Exception e) {
        String type = e.getClass().getName();
        String cause = e.getMessage();
        String trace = Arrays.toString(e.getStackTrace());
        String message = String.format("%s: %s\n%s", type, cause, trace);
        this._log("error", message);
    }

}
