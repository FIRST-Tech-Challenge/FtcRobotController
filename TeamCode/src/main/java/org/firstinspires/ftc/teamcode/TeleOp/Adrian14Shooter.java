package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Adrian14Shooter")
public class Adrian14Shooter extends LinearOpMode {
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    private Servo shoot;

    @Override
    public void runOpMode() {
        BLeft = hardwareMap.dcMotor.get("BLeft");
        BRight = hardwareMap.dcMotor.get("BRight");
        FLeft  = hardwareMap.dcMotor.get("FLeft");
        FRight = hardwareMap.dcMotor.get("FRight");
        shoot = hardwareMap.servo.get("shoot");

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive();
                shootermethod();
            }
        }
    }
    public void shootermethod(){
        if (gamepad1.a){
            shoot.setPosition(1);
        } else if (gamepad1.b) {
            shoot.setPosition(0.5);
        } else if (gamepad1.x){
            shoot.setPosition(0.3);
        } else if (gamepad1.y) {
            shoot.setPosition(0);
        }

    }
    public void drive(){
        if (gamepad1.left_stick_y != 0.0) {
            BLeft.setPower(0.5*gamepad1.left_stick_y);
            BRight.setPower(0.5*gamepad1.left_stick_y);
            FRight.setPower(0.5*gamepad1.left_stick_y);
            FLeft.setPower(0.5*gamepad1.left_stick_y);
        }
        // Strafing
        else if(gamepad1.left_stick_x != 0.0) {
            BLeft.setPower(0.5*gamepad1.left_stick_x);
            BRight.setPower(-0.5*gamepad1.left_stick_x);
            FRight.setPower(0.5*gamepad1.left_stick_x);
            FLeft.setPower(-0.5*gamepad1.left_stick_x);

        }
        // Rotation
        else if (gamepad1.right_stick_x != 0.0) {
            BLeft.setPower(-0.5 * gamepad1.right_stick_x);
            BRight.setPower(0.5 * gamepad1.right_stick_x);
            FRight.setPower(0.5 * gamepad1.right_stick_x);
            FLeft.setPower(-0.5 * gamepad1.right_stick_x);

        }
        else {
            BLeft.setPower(0);
            BRight.setPower(0);
            FRight.setPower(0);
            FLeft.setPower(0);
        }


    }
}
