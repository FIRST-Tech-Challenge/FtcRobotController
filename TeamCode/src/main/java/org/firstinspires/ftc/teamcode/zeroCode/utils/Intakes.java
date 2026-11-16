package org.firstinspires.ftc.teamcode.zeroCode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.Constants;

public class Intakes {
    DcMotor pollenIntake;
    DcMotor nectarIntake;


    public Intakes(){
        pollenIntake = Constants.intakes.intakePollen;
        nectarIntake = Constants.intakes.intakeNectar;
    }
    public void powerPollenIntake(double power) {
        pollenIntake.setPower(power);
    }

    public void powerNectarIntake(double power) {
        nectarIntake.setPower(power);
    }
}
