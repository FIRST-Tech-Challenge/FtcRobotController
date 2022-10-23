package org.firstinspires.ftc.teamcode.robots.taubot.vision.provider.dummy;

import org.firstinspires.ftc.teamcode.robots.taubot.vision.Position;

public class LeftDummyProvider extends AbstractDummyProvider {

    private static final String TELEMETRY_NAME = "Left Dummy Vision Provider";

    @Override
    public Position getPosition() {
        return Position.LEFT;
    }
    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}
