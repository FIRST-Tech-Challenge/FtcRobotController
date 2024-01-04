package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class baseteleop extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_9;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Gyroscope imu_1;
    public Gyroscope imu;


    @Override
    public void runOpMode() {
        //naming motors/sensors
        back_Left = hardwareMap.get(DcMotor.class, "back_Left");
        back_Right = hardwareMap.get(DcMotor.class, "back_Right");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_9 = hardwareMap.get(Blinker.class, "Expansion Hub 9");
        front_Left = hardwareMap.get(DcMotor.class, "front_Left");
        front_Right = hardwareMap.get(DcMotor.class, "front_Right");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            //reversing motors
            //back_Left.setDirection(DcMotor.Direction.REVERSE);
            back_Right.setDirection(DcMotor.Direction.REVERSE);
            //front_Left.setDirection(DcMotor.Direction.REVERSE);
            front_Right.setDirection(DcMotor.Direction.REVERSE);

            //wheels normal drive
            front_Right.setPower(gamepad1.right_stick_y);
            back_Right.setPower(gamepad1.right_stick_y);
            front_Left.setPower(gamepad1.left_stick_y);
            back_Left.setPower(gamepad1.left_stick_y);

            //strafing
            if(gamepad1.right_trigger > .25 ){
                front_Left.setPower(-gamepad1.right_trigger);
                back_Left.setPower(gamepad1.right_trigger);
                front_Right.setPower(gamepad1.right_trigger);
                back_Right.setPower(-gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > .25){
                front_Left.setPower(gamepad1.left_trigger);
                back_Left.setPower(-gamepad1.left_trigger);
                front_Right.setPower(-gamepad1.left_trigger);
                back_Right.setPower(gamepad1.left_trigger);
            };

        }
    }
}