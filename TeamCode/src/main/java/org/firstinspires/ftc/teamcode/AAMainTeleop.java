package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "AA Main Teleop")
public class AAMainTeleop extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorIntake, motorOuttake;
    private CRServo servoDuck;
    private Servo bucket;
    private Robot_2022FF robot;
    private DistanceSensor distsense;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException, TargetPositionNotSetException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        motorIntake = hardwareMap.dcMotor.get("intake");
        motorOuttake = hardwareMap.dcMotor.get("outtake");

        servoDuck = hardwareMap.crservo.get("duck");
        bucket = hardwareMap.servo.get("bucket");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        distsense = hardwareMap.get(DistanceSensor.class, "distsense");

        robot = new Robot_2022FF(motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft, motorIntake, motorOuttake, bucket, servoDuck, distsense, imu, this);
        robot.setupRobot();
        //not here
        //Don't set 0 power behavior or direction here
        //do it in Robot_2022FF

        double powerMod;

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            //the trigger is broken, so using bumper
            //intake
            if (gamepad1.left_bumper) {
                motorIntake.setPower(powerMod);
            }
            else if (gamepad1.right_bumper) {
                motorIntake.setPower(-powerMod);
            }
            else {
                motorIntake.setPower(0);
            }

            //duck
            if (gamepad2.left_bumper) {
                servoDuck.setPower(1);
            }
            else if (gamepad2.right_bumper) {
                servoDuck.setPower(-1);
            }
            else {
                servoDuck.setPower(0);
            }

            //bucket
            if(gamepad2.dpad_up){
                bucket.setPosition(bucket.getPosition()+0.01);
            }
            if(gamepad2.dpad_down){
                bucket.setPosition(bucket.getPosition()-0.01);
            }

            //outtake
            motorOuttake.setPower(gamepad2.right_stick_y/2);

            //trying to keep the arm in position and not swinging around

            //top, middle, bottom scoring
            if (gamepad2.x) {
                robot.dropBottom(powerMod);
            }
            if (gamepad2.a) {
                robot.dropMiddle(powerMod);
            }
            if (gamepad2.b) {
                robot.dropTop(powerMod);
            }


            //drive
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.cos(angle);
            double powerTwo = r*Math.sin(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo - (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo + (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);


            //telemetry
            telemetry.addData("ticks main:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("outtake ticks main:", motorOuttake.getCurrentPosition());
            telemetry.addData("intake ticks", motorIntake.getCurrentPosition());
            telemetry.addData("bucket pos",bucket.getPosition());
            telemetry.update();



        }
    }
}
