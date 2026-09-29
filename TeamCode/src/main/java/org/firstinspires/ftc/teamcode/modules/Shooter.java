package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {
    private DcMotorEx shooterUp;
    private DcMotorEx shooterDown;

    public Shooter(LinearOpMode opMode) {
        shooterUp = opMode.hardwareMap.get(DcMotorEx.class, "shooterUp");
        shooterDown = opMode.hardwareMap.get(DcMotorEx.class, "shooterDown");
    }

    public void setVelocity(double velocity) {
        shooterUp.setVelocity(velocity);
        shooterDown.setVelocity(velocity);
    }

    public double getVelocity() {
      return  shooterUp.getVelocity();
    }
}
