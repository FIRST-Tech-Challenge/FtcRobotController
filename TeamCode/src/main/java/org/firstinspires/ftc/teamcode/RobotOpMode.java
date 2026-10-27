package org.firstinspires.ftc.teamcode;

public class RobotOpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    RobotHardware robot       = new RobotHardware(this);
    TelemetryPrinter telemetryPrinter = new TelemetryPrinter(this);

    public void init() {
        telemetry.addData("Hello", "Chameleons");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Hello", "Chameleons during init_loop");
    }

    @Override
    public void start() {
        double drive = 0;
        telemetry.addData(">", "Starting AutoMode...");

    }

    @Override
    public void loop() {
        telemetry.addData("Hello", "Chameleons on the bot");
    }

    @Override
    public void stop() {
        telemetry.addData(">", "Stopping AutoMode...");
    }
}
