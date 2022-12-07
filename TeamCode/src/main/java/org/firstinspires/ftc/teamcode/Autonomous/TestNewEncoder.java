package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.NewEncoderMove;

@Autonomous(name="EncoderNew", group = "Concept")
//@Disabled
public class TestNewEncoder extends LinearOpMode {

    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack;
    private NewEncoderMove encoderMove;


    @Override
    public void runOpMode()
    {
        leftMotor = hardwareMap.dcMotor.get("FL");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        encoderMove = new NewEncoderMove(leftMotor,leftMotorBack,rightMotor,rightMotorBack);

        waitForStart();

        encoderMove.DriveTo(1000,1000,1000,1000,0.5, opModeIsActive());
        sleep(500);
        encoderMove.DriveTo(-1000,-1000,-1000,-1000,0.5,opModeIsActive());
        sleep(500);
        encoderMove.DriveTo(-1100,-1100,1100,1100,0.5,opModeIsActive());
        sleep(500);
    }
}

