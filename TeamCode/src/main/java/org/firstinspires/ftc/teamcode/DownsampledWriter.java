package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.FlightRecorder;

public class DownsampledWriter {
    public final String channel;
    public final long maxPeriod;
    private long nextWriteTimestamp = System.nanoTime();
    public DownsampledWriter(String channel, long maxPeriod) {
        this.channel = channel;
        this.maxPeriod = maxPeriod;
    }
    public void write(Object msg) {
        long now = System.nanoTime();
        if (now >= nextWriteTimestamp) {
            nextWriteTimestamp = now + maxPeriod;
            FlightRecorder.write(channel, msg);
        }
    }
}
