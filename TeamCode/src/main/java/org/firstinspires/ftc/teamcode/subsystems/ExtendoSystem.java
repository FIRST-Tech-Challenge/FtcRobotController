package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robocol.Command;

import org.jetbrains.annotations.NotNull;

import java.util.Objects;

public class ExtendoSystem extends SubsystemBase {

    final DcMotorEx motor;
    public ExtendoSystem(DcMotorEx motor) {
        this.motor = motor;
    }

    public enum Direction {
        OUTWARD,
        INWARD,
        NONE
    }

    public void setDirection(@NotNull Direction direction) {
        Objects.requireNonNull(direction);
        double power = (direction == Direction.OUTWARD ? 1 : 0) - (direction == Direction.INWARD ? 1 : 0);
        motor.setPower(power);
    }

}
