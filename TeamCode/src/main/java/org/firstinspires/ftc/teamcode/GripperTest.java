package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
public class GripperTest extends OpMode{
    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo leftClaw;
    Servo rightClaw;
    double leftPower; //keep track of the powers
    double rightPower;
    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        leftClaw = hardwareMap.servo.get("left_claw");
        rightClaw = hardwareMap.servo.get("right_claw");

        rightWheel.setDirection(DcMotor.Direction.REVERSE); //Is this line needed? I'm not sure but to turn you need to reverse that side right?
    }

    @Override
    public void loop() {
        leftPower = -gamepad1.left_stick_y;//negate the values because of how the joysticks work
        rightPower = -gamepad1.right_stick_y;
        leftWheel.setPower(leftPower);
        rightWheel.setPower(rightPower);

        if (gamepad1.x){
            rightClaw.setPosition(1);
            leftClaw.setPosition(1);
        }else{
            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
        }
    }
}