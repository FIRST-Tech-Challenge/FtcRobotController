package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class WristSubsystem extends SubsystemBase {

    private final Servo wrist;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("WristSubsystem");

    public WristSubsystem(final Servo wrist) {
        this.wrist = wrist;
    }

    public void moveWrist(double frontward, double backward) {
        dbp.createNewTelePacket();
        double power = frontward-backward;

        dbp.debug(String.format(Locale.ENGLISH, "Power: %f", power), true);
        power++;
        power/=2d;
        wrist.setPosition(power);
    }

    public double getServoPosition() {
        return wrist.getPosition();
    }

}
