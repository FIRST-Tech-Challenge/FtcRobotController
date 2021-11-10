package org.firstinspires.ftc.teamcode.src.Utills;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.PrintWriter;
import java.io.StringWriter;

public class ExceptionPrinter {
    Telemetry telemetry;
    String exceptionString;
    Exception error;

    public ExceptionPrinter(Exception error, Telemetry telemetry) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        error.printStackTrace(pw);
        exceptionString = sw.toString();
        this.telemetry = telemetry;
        this.error = error;
    }

    public void printError(){
        telemetry.addData("Error: ", exceptionString);
        telemetry.update();
    }
}
