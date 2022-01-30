package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;

public interface Subsystem extends TelemetryProvider {
    void update(Canvas fieldOverlay);
    void stop();
}
