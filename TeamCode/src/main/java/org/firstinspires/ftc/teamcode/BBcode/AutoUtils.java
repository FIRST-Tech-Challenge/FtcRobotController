package org.firstinspires.ftc.teamcode.BBcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.PinpointDrive;

public class AutoUtils {
    PinpointDrive drive;
    public AutoUtils(PinpointDrive d) {drive = d;}
    //----------------------------------------------
    public Action Wait(double seconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return drive.actionBuilder(new Pose2d(0,0,0)).waitSeconds(seconds).build().run(telemetryPacket);
            }
        };
    }
}
