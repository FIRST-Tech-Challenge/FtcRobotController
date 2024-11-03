package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class SonicSubsystemBase extends SubsystemBase {
    public void Wait(long timeout) {
        try {
            synchronized (this) {
                wait(timeout);
            }
        } catch (java.lang.InterruptedException e) {
        }
    }
}