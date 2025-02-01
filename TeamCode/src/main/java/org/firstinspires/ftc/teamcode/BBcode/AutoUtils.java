package org.firstinspires.ftc.teamcode.BBcode;

import android.os.SystemClock;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoUtils {
    public static Action Wait(double seconds) {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !(timer.seconds() >= seconds);
            }
        };
    }
}
