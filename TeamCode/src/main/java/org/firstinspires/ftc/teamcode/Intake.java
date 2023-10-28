package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor intakeMotor;
    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveIntake(boolean powered) {
        if (powered) {
            intakeMotor.setPower(1.0);
        }
        else {
            intakeMotor.setPower(0.0);
        }
    }
}
