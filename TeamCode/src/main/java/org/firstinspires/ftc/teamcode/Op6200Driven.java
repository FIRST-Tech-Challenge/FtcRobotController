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

        double drive = -gamepad1.left_stick_y/3.5;
        double strafe = -gamepad1.left_stick_x/3.5;
        double turn = gamepad1.right_stick_x / 1.75;

        double frontLeftPower = Range.clip(drive + strafe + turn, -0.4, 0.4);
        double frontRightPower = Range.clip(drive - strafe - turn, -0.4, 0.4);
        double backLeftPower = Range.clip(drive - strafe + turn, -0.4, 0.4);
        double backRightPower = Range.clip(drive + strafe - turn, -0.4, 0.4);

        robot.leftFront.setPower(frontLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.leftRear.setPower(backLeftPower);
        robot.rightRear.setPower(backRightPower);
        /*if(gamepad1.right_trigger > gamepad1.left_trigger){
            robot.servo.setDirection(Servo.Direction.FORWARD);
            robot.servo.setPosition(gamepad1.right_trigger);
        }else if(gamepad1.left_trigger > gamepad1.right_trigger){
            robot.servo.setDirection(Servo.Direction.REVERSE);
            robot.servo.setPosition(gamepad1.left_trigger);
        }*/
        double gripPos = robot.servo.getPosition();
        if(gamepad1.left_trigger > 0 && gripPos > minPosition){
            gripPos = gripPos - 0.01;
        }else if(gamepad1.right_trigger > 0 && gripPos < maxPosition){
            gripPos = gripPos + 0.01;
        }else{
            gripPos = gripPos;
        }


        robot.servo.setPosition(Range.clip(gripPos, minPosition, maxPosition));

        /*float lt = gamepad1.left_trigger / 2;
        float rt = gamepad1.right_trigger / 2;
        if(gamepad1.left_trigger != 0){
            robot.servo.setDirection(Servo.Direction.REVERSE);
            robot.servo.setPosition(lt);
        }else if(gamepad1.right_trigger != 0){
            if(robot.servo.getPosition() < 0.5){
                robot.servo.setDirection(Servo.Direction.FORWARD);
                robot.servo.setPosition(rt);
            }

        }*/
/*

        if(gamepad1.right_trigger != 0)
        {
            robot.servo.setDirection(Servo.Direction.FORWARD);


                robot.servo.setPosition(1);

            }

        if(gamepad1.left_trigger != 0)
        {
            //robot.servo.setDirection(Servo.Direction.REVERSE);
            robot.servo.setPosition(0.0);
            telemetry.addData("releasing cone servo position", robot.servo.getPosition());
        }

*/


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


