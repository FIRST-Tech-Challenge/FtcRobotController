package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="testShooterLoading")
public class TestShooterLoading extends LinearOpMode{
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.shooterServo1(.7);
        robot.shooterServo1(.7);
        robot.intakeEngage(.7);
    }
}

