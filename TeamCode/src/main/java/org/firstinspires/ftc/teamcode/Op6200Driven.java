package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.Team6200.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



@TeleOp
public class Op6200Driven extends OpMode {
    private Movement movement;
    private SampleMecanumDrive robot;

    @Override
    public void init() {

        //movement = new Movement(hardwareMap);
        robot = new SampleMecanumDrive(hardwareMap);

    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x / 1.75;

        double frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        double frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        double backLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
        robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
        robot.rightRear.setDirection(DcMotor.Direction.FORWARD);
        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightRear.setPower(backRightPower);

        if(gamepad1.right_bumper)
        {
            robot.servo.setPosition(15);
            telemetry.addData("servo position", robot.servo.getPosition());
        }

        if(gamepad1.left_bumper)
        {
            robot.servo.setDirection(Servo.Direction.REVERSE);
            robot.servo.setPosition(0);
            telemetry.addData("servo position", robot.servo.getPosition());
        }




        //manual movement for linear slide
        //int lmotorpos = robot.lmotor.getCurrentPosition();
        ///if(gamepad1.right_trigger != 0)
       // {
//            robot.lmotor.setTargetPosition(lmotorpos + 60);
//            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lmotor.setPower(0.8);
//            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());
//
//        }
//
//        if(gamepad1.left_trigger != 0)
//        {
//
//            robot.lmotor.setTargetPosition(lmotorpos - 60);
//            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lmotor.setPower(0.8);
//            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());
//
//        }
    }
}


