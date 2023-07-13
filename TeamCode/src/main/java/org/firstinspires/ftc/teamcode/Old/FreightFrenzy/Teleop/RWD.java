package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RWD")
@Disabled
public class RWD extends LinearOpMode
{
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
//    private DcMotor motorLeftFront;
//    private DcMotor motorRightFront;
    boolean isTurning = true;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");
//        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
//        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        resetRuntime();
        while(opModeIsActive()&&getRuntime()<90)
        {
            motorRightBack.setPower(gamepad1.left_stick_y*0.15+gamepad1.right_stick_x*0.2);
            motorLeftBack.setPower(gamepad1.left_stick_y*0.15-gamepad1.right_stick_x*0.2);

            idle();
        }
    }
}