package org.firstinspires.ftc.teamcode.BioBuzzTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.ByteBustersBee;

@TeleOp
public class    BioBuzzTeleOp extends OpMode
{
    ByteBustersBee byteBustersBee = new ByteBustersBee();

    @Override
    public void init()
    {
        byteBustersBee.init(hardwareMap);
    }

    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;
       // byteBustersBee.setFrontRightMotorSpeed(motorSpeed);

        byteBustersBee.setFrontRightMotorSpeed(motorspeed);
        telemetry.add("Motor Speed:",motorSpeed);
        telemetry.addData("Motor Revs", byteBustersBee.getFrontMotorRevs());

        if(gamepad1.a)
        {
            byteBustersBee.setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if(gamepad1.b)
        {
            byteBustersBee.setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
        }
//Change if condition here.
        if (gamepad1.y)
        {
            byteBustersBee.setBackRightMotorSpeed((0.5));
        }
        }
}




