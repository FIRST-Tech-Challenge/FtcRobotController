package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriftForWyatt extends OpMode {

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    public void moveRobot(){
        double vertical;
        double horizontal;
        double pivot;

        vertical = -0.75 * gamepad1.left_stick_y;
        horizontal = 0.75 * gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void init(){

        BLeft = hardwareMap.dcMotor.get("Bleft");
        BRight = hardwareMap.dcMotor.get("Bright");
        FLeft  = hardwareMap.dcMotor.get("Fleft");
        FRight = hardwareMap.dcMotor.get("Fright");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();
    }

}
