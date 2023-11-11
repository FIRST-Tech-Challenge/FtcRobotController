package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriftForWyatt extends OpMode {

    private DcMotor Bleft;
    private DcMotor Bright;
    private DcMotor Fleft;
    private DcMotor Fright;

    public void moveRobot(){
        double vertical;
        double horizontal;
        double pivot;

        vertical = 0.75 * gamepad1.left_stick_y;
        horizontal = -0.75 * gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        Fright.setPower((pivot + (-vertical + horizontal)));
        Bright.setPower(pivot + (-vertical - horizontal));
        Fleft.setPower((-pivot + (-vertical - horizontal)));
        Bleft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void init(){

        Bleft = hardwareMap.dcMotor.get("Bleft");
        Bright = hardwareMap.dcMotor.get("Bright");
        Fleft  = hardwareMap.dcMotor.get("Fleft");
        Fright = hardwareMap.dcMotor.get("Fright");

        Bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Fleft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();
    }

}
