package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    DcMotor outtakeMotor;

    public Outtake(HardwareMap hwMap) {
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveLift(double power) {
        outtakeMotor.setPower(power);
        }
    }
