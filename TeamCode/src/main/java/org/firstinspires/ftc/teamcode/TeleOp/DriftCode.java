package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriftCode extends OpMode {

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor Arm;
    private DcMotor Linear;
    private Servo Claw;
    private Servo shoot;

    public void moveRobot(){
        double vertical;
        double horizontal;
        double pivot;

        vertical = 0.75 * gamepad1.left_stick_y;
        horizontal = -0.75 * gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void init(){

        BLeft = hardwareMap.dcMotor.get("BLeft");
        BRight = hardwareMap.dcMotor.get("BRight");
        FLeft  = hardwareMap.dcMotor.get("FLeft");
        FRight = hardwareMap.dcMotor.get("FRight");
        Arm = hardwareMap.dcMotor.get("Arm");
        Linear = hardwareMap.dcMotor.get("Linear");
        Claw = hardwareMap.servo.get("Claw");

        BRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();

        if (gamepad2.left_stick_y != 0.0) {
            Arm.setPower(gamepad2.left_stick_y);

        }

        else if (gamepad2.right_stick_y != 0.0) {
            Linear.setPower(0.5 * gamepad2.right_stick_y);

        }

        else if (gamepad2.b) {
            Claw.setPosition(1.0);

        }

        else if (gamepad2.x) {
            Claw.setPosition(0.0);

        }
    }


}
