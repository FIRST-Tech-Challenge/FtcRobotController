package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;
    private final boolean continuousMode;
    private boolean zeroPositionMode = false;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final Servo wrist, boolean continuousMode) {
        this.wrist = wrist;
        this.continuousMode = continuousMode;
    }

    public void moveWrist(double frontward, double backward) {

        double power = frontward - backward;
        dbp.debug(String.format(Locale.ENGLISH, "Power: %f", power), true);
        dbp.debug("Servo position: " + getServoPosition(), true);

        if (continuousMode) {
            power++;
            power /= 2d;
            wrist.setPosition(power);
        } else {
            // If frontward button is selected, then
            float deadzoneOffset = .1f;
            if (frontward > deadzoneOffset) {
                zeroPositionMode = false;
            }
            if (backward > deadzoneOffset) {
                zeroPositionMode = true;
            }
            if (zeroPositionMode) {
                // Set the position to 0
                wrist.setPosition(0);
            } else {
                // Set the position to 1 (board location
                wrist.setPosition(1);
            }
        }
    }

    public double getServoPosition() {
        return wrist.getPosition();
    }

}
