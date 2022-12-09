package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.Team6200.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



@TeleOp
public class Op6200Driven extends OpMode {
    float minPosition = 0.3f;
    float maxPosition = 0.8f;

    private SampleMecanumDrive robot;

    @Override
    public void init() {

        //movement = new Movement(hardwareMap);
        robot = new SampleMecanumDrive(hardwareMap);


    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;



        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 0.60);
        double frontLeftPower = (drive + strafe + turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;

        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightRear.setPower(backRightPower);

        double gripPos = robot.servo.getPosition();
        if(gamepad1.left_trigger > 0 && gripPos > minPosition){
            gripPos = minPosition;
        }else if(gamepad1.right_trigger > 0 && gripPos < maxPosition){
            gripPos = maxPosition;
        }else{
            gripPos = gripPos;
        }


        robot.servo.setPosition(Range.clip(gripPos, minPosition, maxPosition));



        telemetry.addData("general servo position", robot.servo.getPosition());




        //manual movement for linear slide
        int lmotorpos = robot.lmotor.getCurrentPosition();
        if(gamepad1.right_bumper)
        {
            robot.lmotor.setTargetPosition(lmotorpos + 60);
            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lmotor.setPower(0.95);
            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());

        }

        if(gamepad1.left_bumper)
        {
            robot.lmotor.setTargetPosition(lmotorpos - 60);
            robot.lmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lmotor.setPower(0.95);
            telemetry.addData("Current position", robot.lmotor.getCurrentPosition());

        }}
}

