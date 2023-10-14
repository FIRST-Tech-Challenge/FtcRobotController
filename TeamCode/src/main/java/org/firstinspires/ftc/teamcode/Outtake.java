package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    DcMotor outtakeMotor;

    public Outtake(HardwareMap hwMap) {
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void driveLift(boolean up, boolean down) {
        if (up) {
            outtakeMotor.setPower(1.0);
        }
        else if (down) {
            outtakeMotor.setPower(-1.0);
        }
        else {
            outtakeMotor.setPower(0.0);
        }
    }

}
