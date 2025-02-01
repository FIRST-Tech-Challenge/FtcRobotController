package org.firstinspires.ftc.teamcode.BBcode.UtilClasses;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class UtilActions {
    public static Action Wait(double seconds) {
        return new Action() {
            ElapsedTime timer = null;
            @Override
            public boolean run (@NonNull TelemetryPacket telemetryPacket) {
                if (timer == null) {timer = new ElapsedTime();}
            return !(timer.seconds() >= seconds);
            }
        };
    }
}
