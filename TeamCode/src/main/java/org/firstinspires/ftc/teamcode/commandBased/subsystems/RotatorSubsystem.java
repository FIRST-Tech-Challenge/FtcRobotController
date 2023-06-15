package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RotatorSubsystem extends SubsystemBase {

    private ServoImplEx rotator;

    public RotatorSubsystem(final HardwareMap hwMap) {

        rotator = hwMap.get(ServoImplEx.class, "rotator");
        rotator.setDirection(Servo.Direction.REVERSE);
    }

    public void enable() {
        rotator.setPwmEnable();
    }

    public void disable() {
        rotator.setPwmDisable();
    }

    public void setPosition(double pos) {
        rotator.setPosition(pos);
    }

    public void setPWMRange(PwmControl.PwmRange range) {
        rotator.setPwmRange(range);
    }

    public double getPosition() {
        return rotator.getPosition();
    }

    public double[] getPWMRange() {
        PwmControl.PwmRange range = rotator.getPwmRange();
        return new double[]{range.usFrame, range.usPulseLower, range.usPulseUpper};
    }

}
