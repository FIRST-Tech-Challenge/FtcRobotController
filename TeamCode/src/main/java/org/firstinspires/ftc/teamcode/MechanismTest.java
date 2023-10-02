package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Mechanism Test", group = "Teleop")
public class MechanismTest extends OpMode
{
    public CRServo lWheel;
    public CRServo rWheel;

    @Override
    public void init() //initialization method
    {
        lWheel = hardwareMap.crservo.get("Left Wheel");
        rWheel = hardwareMap.crservo.get("Right Wheel");
    }

    @Override
    public void loop() //teleop loop
    {
        if(gamepad2.right_trigger == 0)
        {
            rWheel.setPower(-gamepad2.left_trigger);
            lWheel.setPower(gamepad2.left_trigger);
        }
        else
        {
            rWheel.setPower(gamepad2.right_trigger);
            lWheel.setPower(-gamepad2.right_trigger);
        }

        telemetry.addLine("looping :)");
        telemetry.update();
    }
}
