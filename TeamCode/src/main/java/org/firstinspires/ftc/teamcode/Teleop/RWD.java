package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RWD")
public class RWD extends LinearOpMode
{
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
    boolean isTurning = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        while(opModeIsActive())
        {
            motorRightBack.setPower(gamepad1.left_stick_y*0.7-gamepad1.right_stick_x);
            motorLeftBack.setPower(gamepad1.left_stick_y*0.7+gamepad1.right_stick_x);

            idle();
        }
    }
}