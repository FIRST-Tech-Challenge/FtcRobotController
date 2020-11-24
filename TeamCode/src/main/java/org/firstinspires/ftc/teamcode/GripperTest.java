package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="GripperTest", group="Pushbot")
public class GripperTest extends OpMode{
    DcMotor fleftWheel;
    DcMotor frightWheel;
    DcMotor rleftWheel;
    DcMotor rrightWheel;
    Servo leftClaw;
    Servo rightClaw;
    double leftPower; //keep track of the powers
    double rightPower;
    @Override
    public void init() {
        fleftWheel = hardwareMap.dcMotor.get("frontleft");
        frightWheel = hardwareMap.dcMotor.get("frontright");
        rleftWheel = hardwareMap.dcMotor.get("backleft");
        rrightWheel = hardwareMap.dcMotor.get("backright");
        leftClaw = hardwareMap.servo.get("left_claw");
        rightClaw = hardwareMap.servo.get("right_claw");

        frightWheel.setDirection(DcMotor.Direction.REVERSE);
        rrightWheel.setDirection(DcMotor.Direction.REVERSE);//Is this line needed? I'm not sure but to turn you need to reverse that side right?
    }

    @Override
    public void loop() {
        leftPower = -gamepad1.left_stick_y;//negate the values because of how the joysticks work
        rightPower = -gamepad1.right_stick_y;
        fleftWheel.setPower(leftPower);
        frightWheel.setPower(rightPower);
        rleftWheel.setPower(leftPower);
        rrightWheel.setPower(rightPower);

        if (gamepad1.x){
            rightClaw.setPosition(1);
            leftClaw.setPosition(1);
        }else{
            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
        }
    }
}