package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.provider.dummy;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;

public class RightDummyProvider extends AbstractDummyProvider {

    private static final String TELEMETRY_NAME = "Right Dummy Vision Provider";

    @Override
    public Position getPosition() {
        return Position.RIGHT;
    }
    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }
}
