package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.MatchRecorder.MatchLogger;
import org.opencv.core.Mat;

public class LauncherSubsystem extends SubsystemBase {

    final Servo servo;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("Launcher Subsystem");

    public LauncherSubsystem(final Servo servo) {
        this.servo = servo;
    }

    public void setPosition(LauncherPosition position) {
        if (servo == null) {
            dbp.info("WARNING: Launcher servo is not set up in the config!", true);
            return;
        }
        servo.setPosition(position.getPosition());
        MatchLogger.getInstance().genericLog("Launcher", MatchLogger.FileType.LAUNCHER, position.name(), position.getPosition());
    }

    public enum LauncherPosition {
        ZERO(0),
        ACTIVATE(1f);

        float position;
        LauncherPosition(float position) {
            this.position = position;
        }

        public float getPosition() {
            return position;
        }
    }

}
