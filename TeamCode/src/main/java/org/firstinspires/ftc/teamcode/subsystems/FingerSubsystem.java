package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

public class FingerSubsystem extends SubsystemBase {

    Servo finger;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("FingerSubsystem");

    public FingerSubsystem(final Servo finger) {
        this.finger = finger;
    }

    public void locomoteFinger() {

    }

}
