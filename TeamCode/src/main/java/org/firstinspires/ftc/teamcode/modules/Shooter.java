package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {
    private DcMotorEx shooterUp;
    private DcMotorEx shooterDown;
    public double RPS = 50;
    public double TPR = 28;

    public Shooter(LinearOpMode opMode) {
        shooterUp = opMode.hardwareMap.get(DcMotorEx.class, "shooterUp");
        shooterDown = opMode.hardwareMap.get(DcMotorEx.class, "shooterDown");
    }

    public void setVelocity(double velocity) {
        shooterUp.setVelocity(velocity);
        shooterDown.setVelocity(velocity);
    }

    public double getVelocity() {
      return  convertTPStoRPS(shooterUp.getVelocity());
    }

    public double convertRPStoTPS(double rps) {
        return rps*TPR;
    }
    public double convertTPStoRPS(double tps) {
        return tps/TPR;
    }

}
