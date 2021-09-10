package org.firstinspires.ftc.teamcode.RobotData;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Good Bot TeleOp ", group = "Opmode")
public class Goodbot_TeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftRight;
    private DcMotor rightFront;
    private DcMotor rightRear;
    @Override
    public void runOpMode() throws InterruptedException
    {
            waitForStart();

            while(opModeIsActive())
            {
                idle();
            }

    }
}
