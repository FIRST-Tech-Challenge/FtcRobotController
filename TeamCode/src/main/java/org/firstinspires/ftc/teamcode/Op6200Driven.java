package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.Team6200.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@TeleOp
public class Op6200Driven extends OpMode {
    SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
    private Movement movement;

    @Override
    public void init() {

        movement = new Movement(hardwareMap);
    }

    @Override
    public void loop() {

        movement.processJoystick(gamepad1.left_stick_x, 0 - gamepad1.left_stick_y, gamepad1.right_stick_x);

        //manual movement for linear slide
        int lmotorpos = robot.lmotor.getCurrentPosition();
        if(gamepad1.right_trigger != 0)
        {
            robot.lmotor.setTargetPosition(lmotorpos + 60);
            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lmotor.setPower(0.8);
            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());

        }

        if(gamepad1.left_trigger != 0)
        {

            robot.lmotor.setTargetPosition(lmotorpos - 60);
            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lmotor.setPower(0.8);
            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());

        }
    }
}


