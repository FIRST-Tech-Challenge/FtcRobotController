package org.firstinspires.ftc.teamcode.disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class telemetryList {
    Telemetry t;
    ArrayList<Object> items = new ArrayList<>();

    public telemetryList(Telemetry telemetry) {
        t = telemetry;
    }

    public void add(String s) {
        Telemetry.Line i = t.addLine(s);
        items.add(i);
    }

    public void addData(String s, Object o) {

    }
}
