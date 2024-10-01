package org.firstinspires.ftc.teamcode.learning;

import com.parshwa.drive.Drive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.nio.file.Files;

@TeleOp(name = "Aarav's Teleop")
public class drive2 extends LinearOpMode {
    private DcMotor LFM;
    private DcMotor LBM;
    private DcMotor RFM;
    private DcMotor RBM;
    private RevHubOrientationOnRobot orientation;
    private IMU imu;
    private double SPEED =0;
    private Drive driver = new Drive();
    @Override
    public void runOpMode() throws InterruptedException {
        LFM = hardwareMap.dcMotor.get("LFM");
        LBM = hardwareMap.dcMotor.get("LBM");
        RFM = hardwareMap.dcMotor.get("RFM");
        RBM = hardwareMap.dcMotor.get("RBM");
        LFM.setDirection(DcMotorSimple.Direction.FORWARD);
        LBM.setDirection(DcMotorSimple.Direction.REVERSE);
        RFM.setDirection(DcMotorSimple.Direction.FORWARD);
        RBM.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("initialized");
        telemetry.update();
        imu.initialize(new IMU.Parameters(orientation));
        waitForStart();
        while (!isStopRequested()) {
            SPEED = gamepad1.right_trigger / 2.0;
            if(gamepad1.right_bumper){
                SPEED = SPEED * 2.0;
            }

            driver.move(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,SPEED);
        }


    }

    private void setSpeed(){}
}