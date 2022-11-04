package org.firstinspires.ftc.teamcode.dragonswpilib;

public class SubsystemBase implements Subsystem {

    /** Constructor. */
    public SubsystemBase() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

}
