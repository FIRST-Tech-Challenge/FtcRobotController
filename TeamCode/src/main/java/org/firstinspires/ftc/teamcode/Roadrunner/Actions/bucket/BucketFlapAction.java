package org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class BucketFlapAction implements Action {
    private ViperSlide viperSlide;
    private String position;
    private int sleepTime;

    public BucketFlapAction(ViperSlide viperSlide, String position, int sleepTime) {
        this.viperSlide = viperSlide;
        this.position = position;
        this.sleepTime = sleepTime;
    }

    public BucketFlapAction(ViperSlide viperSlide, String position) {
        this(viperSlide, position, 0);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if ("close".equals(position)) {
            viperSlide.closeBucket();
        } else if ("open".equals(position)) {
            viperSlide.openBucket();
            sleep(sleepTime);
        }

        return false;
    }
}