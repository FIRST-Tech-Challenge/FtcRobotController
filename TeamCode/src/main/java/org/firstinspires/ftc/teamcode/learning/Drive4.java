package org.firstinspires.ftc.teamcode.learning;

import com.parshwa.drive.Drive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name="Ranveers TeleOp")
public class Drive4 extends LinearOpMode {
    private Drive driver = new Drive();
    private RevHubOrientationOnRobot orientation;
    private IMU imu;
    private double SPED = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = hardwareMap.get(IMU.class, "imu");
        if (orientation == null) {
            orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        }
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("initialized");
        telemetry.update();
        waitForStart();
        while (!isStopRequested()){
            SPED = gamepad1.right_trigger / 2.0;
            if(gamepad1.right_bumper){
                SPED = SPED * 2.0;
            }
            driver.move(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,SPED);
        }
    }
    private void setSpeed(){}
\}