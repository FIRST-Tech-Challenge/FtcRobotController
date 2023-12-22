package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Objects;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final Servo wrist) {
        this.wrist = wrist;
    }

    public enum WristState {
        OPEN(0.25),
        PINCH(0);

        public double position;

        WristState(double position) {
            this.position = position;
        }

    }

    public void setWristState(WristState state) {
        Objects.requireNonNull(state);
        wrist.setPosition(state.position);
    }

}
