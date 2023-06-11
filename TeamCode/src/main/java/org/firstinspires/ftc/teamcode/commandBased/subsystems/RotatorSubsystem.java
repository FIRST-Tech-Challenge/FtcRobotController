package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class RotatorSubsystem extends SubsystemBase {

    private ServoEx rotator;

    public RotatorSubsystem(final HardwareMap hwMap) {

        rotator = new SimpleServo(
                hwMap,
                "rotator",
                Constants.ROTATOR_MIN,
                Constants.ROTATOR_MAX
        );
    }

    public void setRotation(double angle) {
        rotator.turnToAngle(angle);
    }

    public double getAngle() {
        return rotator.getAngle();
    }

    public double getPosition() {
        return rotator.getPosition();
    }
}
