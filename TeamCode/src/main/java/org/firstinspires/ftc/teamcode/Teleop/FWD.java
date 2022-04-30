package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "FWD")
public class FWD extends LinearOpMode
{
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    boolean isTurning = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");

        waitForStart();

        resetStartTime();
        while(opModeIsActive()&&getRuntime()<90)
        {
            motorRightBack.setPower(gamepad1.left_stick_y*0.3+gamepad1.right_stick_x*0.4);
            motorLeftBack.setPower(gamepad1.left_stick_y*0.3-gamepad1.right_stick_x*0.4);
            motorRightFront.setPower(gamepad1.left_stick_y*0.3+gamepad1.right_stick_x*0.4);
            motorLeftFront.setPower(gamepad1.left_stick_y*0.3-gamepad1.right_stick_x*0.4);

            idle();
        }
    }
}