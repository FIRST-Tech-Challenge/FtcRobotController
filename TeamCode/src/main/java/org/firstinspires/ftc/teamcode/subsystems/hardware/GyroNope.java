package org.firstinspires.ftc.teamcode.subsystems.hardware;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public interface GyroNope  extends SubSystem {
    void resetAngle();
    double getAngle();
    void turnTo(double degrees);
    double getAbsoluteAngle();
}
