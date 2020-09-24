package com.hfrobots.tnt.season1920;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A tele-op that runs our normal tele-op for the season, but
 * sends metrics without having to go through ChaosNinja, and doesn't
 * let the robot move on the floor, since it may be on a table or stand
 */
@TeleOp(name="Metrics Demo", group="util")
@SuppressWarnings("unused")
public class MetricDemoTeleop extends SkystoneTeleop {
    @Override
    public void start() {
        super.start();

        setupMetricsSampler();
    }

    @Override
    public void loop() {
        skystoneGrabber.stow(); // keep the grabber stowed at all times in tele-op

        operatorControls.periodicTask();

        if (metricSampler != null) {
            metricSampler.doSamples();
        }

        updateTelemetry(telemetry);
    }
}
