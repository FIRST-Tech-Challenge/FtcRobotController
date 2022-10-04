package org.firstinspires.ftc.teamcode.robots.gruntbuggly.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.TelemetryProvider;

/**
 * @author Mahesh Natamai
 */

public interface Subsystem extends TelemetryProvider {
    void update(Canvas fieldOverlay);
    void stop();
}
