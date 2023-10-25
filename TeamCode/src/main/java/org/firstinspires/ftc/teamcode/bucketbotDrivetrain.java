package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class bucketbotDrivetrain {
    DcMotor rightMotor, leftMotor, frontMotor, backMotor;


    public bucketbotDrivetrain(HardwareMap hwMap) {
        rightMotor = hwMap.get(DcMotor.class, "right");
        leftMotor = hwMap.get(DcMotor.class, "left");
        frontMotor = hwMap.get(DcMotor.class, "front");
        backMotor = hwMap.get(DcMotor.class, "back");


        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontMotor.setDirection(DcMotor.Direction.FORWARD);
        backMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double x, double y) {
        rightMotor.setPower(x);
        leftMotor.setPower(x);

        frontMotor.setPower(y);
        backMotor.setPower(y);
    }

}
