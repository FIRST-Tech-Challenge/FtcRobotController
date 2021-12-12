package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.Utills.ThreadedSubsystemTemplate;

public class LEDs extends ThreadedSubsystemTemplate {
    public LEDs(Executable<Boolean> opModeIsActive, Executable<Boolean> isStopRequested) {
        super(opModeIsActive, isStopRequested);
    }

    protected void threadMain() {

    }
}
