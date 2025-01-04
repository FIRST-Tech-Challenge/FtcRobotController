package org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.ViperSlide;

public class ViperDownForTimeAction implements Action {
    private ViperSlide viperSlide;
    private int time;

    public ViperDownForTimeAction(ViperSlide viperSlide, int time) {
        this.viperSlide = viperSlide;
        this.time = time;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        viperSlide.setPower(-1);
        sleep(time);
        viperSlide.stop();
        return false;
    }
}
