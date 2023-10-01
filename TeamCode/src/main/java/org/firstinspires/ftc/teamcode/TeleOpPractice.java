package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.FormatterClosedException;

@TeleOp(name="TestOpMode", group="blah")
public class TeleOpPractice extends LinearOpMode {

    robotHardware robot = new robotHardware(this);

    @Override
    public void runOpMode(){

        robot.init();
        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("status","started");
            telemetry.update();

            robot.driveRobot(gamepad1);

        }
    }





}
