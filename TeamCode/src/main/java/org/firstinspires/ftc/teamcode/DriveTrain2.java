/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain2 extends OpMode {

    HardwareMap hardwareMap;



    double x = gamepad1.left_stick_x;
    double y = gamepad1.left_stick_y;
    double rx = gamepad1.right_stick_x;


    public void init() {

        left_f = hardwareMap.get(DcMotorEx.class, "left_front");
        right_f = hardwareMap.get(DcMotorEx.class, "right_front");
        left_b = hardwareMap.get(DcMotorEx.class, "left_back");
        right_b = hardwareMap.get(DcMotorEx.class, "right_back");

        right_f.setDirection(DcMotorSimple.Direction.REVERSE);
        right_b.setDirection(DcMotorSimple.Direction.REVERSE);

    }





    @Override
    public void loop() {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        right_f.setPower( (y+ x + rx) / denominator);
        left_b.setPower( (y -x + rx) / denominator);
        left_f.setPower( (y - x - rx) / denominator);
        right_b.setPower( (y + x - rx) / denominator);

    }


}


 */




