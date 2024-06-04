package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Placer;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.rustlib.commandsystem.Command;
import org.rustlib.drive.MecanumBase;
import org.rustlib.drive.Waypoint;

import java.util.function.Supplier;

public class BackdropHome extends Command {
    private final MecanumBase base;
    private final Slide slide;
    private final Placer placer;
    private final Supplier<Waypoint> futureBackdropWaypoint;
    private final double followTimeout;
    private final double endTime;
    private Waypoint backdropWaypoint;
    private boolean atWaypoint = false;
    private boolean lastAtWaypoint = false;
    private double timestamp = -1;

    public BackdropHome(MecanumBase base, Slide slide, Placer placer, Supplier<Waypoint> futureBackdropWaypoint, double followTimeout, double endTime) {
        this.futureBackdropWaypoint = futureBackdropWaypoint;
        this.base = base;
        this.slide = slide;
        this.placer = placer;
        this.followTimeout = followTimeout;
        this.endTime = endTime;
    }

    @Override
    public void initialize() {
        atWaypoint = false;
        lastAtWaypoint = false;
        backdropWaypoint = futureBackdropWaypoint.get();
    }

    @Override
    public void execute() {
        base.driveToPosition(backdropWaypoint);
    }

    @Override
    public boolean isFinished() {
        lastAtWaypoint = atWaypoint;
        atWaypoint = base.atWaypoint(backdropWaypoint, 0.5, 5)
                || timeSinceInitialized() > followTimeout
                || placer.touchSensor.isPressed()
                || (placer.getDistance() < 4.0 && slide.encoder.getPosition() > 300);

        if (atWaypoint && !lastAtWaypoint) {
            timestamp = timeSinceInitialized();
        }

        return atWaypoint && timeSinceInitialized() > timestamp + endTime;
    }
}
