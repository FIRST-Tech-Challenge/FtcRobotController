package org.firstinspires.ftc;

@OpMode(name="Driving Mode")
public class DriverOpMode extends OpMode {
    
    SpudBot robot;

    @Override
    public void init() {
        robot = SpudBot.get();
    }
    
    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");
        
    }

}
