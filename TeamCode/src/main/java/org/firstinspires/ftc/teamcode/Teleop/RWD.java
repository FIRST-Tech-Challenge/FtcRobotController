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
            //forward
            if (gamepad1.left_stick_y > 0) {
                motorLeftBack.setPower(0.7);
                motorRightBack.setPower(0.7);
                isTurning = true;
            }
            //back
            if (gamepad1.left_stick_y < 0) {
                motorLeftBack.setPower(-0.7);
                motorRightBack.setPower(-0.7);
                isTurning = true;
            }
            //right
            if (gamepad1.right_stick_x > 0 && isTurning) {
                motorLeftBack.setPower(-0.6);
                motorRightBack.setPower(0.6);
            }
            //left
            if (gamepad1.right_stick_x < 0 && isTurning) {
                motorLeftBack.setPower(0.6);
                motorRightBack.setPower(-0.6);
            }
            isTurning = true;
            motorLeftBack.setPower(0);
            motorRightBack.setPower(0);

            idle();
        }
    }
}