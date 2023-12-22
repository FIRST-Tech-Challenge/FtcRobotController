package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor armMotor;

    public ArmSubsystem(final HardwareMap hMap, final String armName) {
        armMotor = hMap.get(DcMotor.class, armName);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
