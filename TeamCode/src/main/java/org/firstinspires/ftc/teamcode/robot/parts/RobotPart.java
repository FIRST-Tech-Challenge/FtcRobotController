package org.firstinspires.ftc.teamcode.robot.parts;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Parts of a robot; Subsystem of a robot.
 * @see RobotPartSettings
 * @author 22jmiller
 */
public abstract class RobotPart {
    private Telemetry telemetry;
    private boolean useTelemetry;


    public void useTelemetry(String caption, Object value) {
        if (useTelemetry) telemetry.addData(caption, value);
    }

    public abstract void start();
    public abstract void stop();
    public abstract void loop();
}
