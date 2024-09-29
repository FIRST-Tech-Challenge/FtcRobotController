package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name= "hoco")
public class jarlsberg extends OpMode{

    DcMotorEx rightFront = null;
    DcMotorEx leftFront = null;
    DcMotorEx rightBack = null;
    DcMotorEx leftBack = null;

    @Override
    public void init()
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {

        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float rx = gamepad1.right_stick_x;

        leftFront.setPower(y+x+rx);
        rightFront.setPower(y-x-rx);
        leftBack.setPower(y-x+rx);
        rightBack.setPower(y+x-rx);

    }


}
