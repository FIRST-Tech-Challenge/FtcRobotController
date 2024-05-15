package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Suspension {

    private DcMotor susMotor;

    public static double SPEED = 1;

    public Suspension(LinearOpMode pup) {
        susMotor = pup.hardwareMap.get(DcMotor.class, "SusMotor");
    }

    public void moveUp() {
        susMotor.setPower(SPEED);

    }

    public void moveDown() {
        susMotor.setPower(-SPEED);

    }

    public void stop() {
        susMotor.setPower(0);

    }

}
