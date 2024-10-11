package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract public class TestItem {
    private String description;

    protected TestItem(String description){
        this.description = description;
    }

    public String getDescription(){
        return description;
    }

    abstract public void run(boolean on, Telemetry telemetry);
}
