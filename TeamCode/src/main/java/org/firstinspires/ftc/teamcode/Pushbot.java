package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "pushbot")
public class Pushbot extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private BNO055IMU imu;
    private Robot_2022FF robot;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new Robot_2022FF(motorFrontRight,motorFrontLeft,motorBackRight,motorBackLeft,imu,this);
        robot.setupRobot();

        waitForStart();
        robot.resetAngle();
        while(opModeIsActive()) {
            double dist = robot.getDistanceTraveled();
            double error = 50-dist;
            telemetry.addData("current position", dist);
            telemetry.addData("error", Math.abs(error));

            double angle = robot.getAngle();
            telemetry.addData("angle", angle);

            telemetry.update();
        }

    }
}


