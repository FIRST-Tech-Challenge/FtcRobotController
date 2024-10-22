package org.firstinspires.ftc.teamcode.COMPETITIONCODE;

import com.parshwa.drive.tele.Drive;
import com.parshwa.drive.tele.DriveModes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "teleop")
public class Teleop extends LinearOpMode {
    private Drive driver = new Drive();
    private double SPED = 0;
    private IMU imu;
    private RevHubOrientationOnRobot orientation;
    @Override
    public void runOpMode() throws InterruptedException {
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE);
        driver.init(hardwareMap,telemetry, DriveModes.MecanumFeildOriented);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()){
            SPED = gamepad1.right_trigger / 2.5;
            if(gamepad1.right_bumper){
                SPED = SPED * 2.0;
            }
            double forward = gamepad1.a ? 0.0 : gamepad1.left_stick_y;
            double strafe = gamepad1.b ? 0.0 : -gamepad1.left_stick_x;
            driver.move(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x,SPED);
        }
        File ThreadManger = AppUtil.getInstance().getSettingsFile("ThreadManger.txt");
        ReadWriteFile.writeFile(ThreadManger, "STOP");

    }
}
