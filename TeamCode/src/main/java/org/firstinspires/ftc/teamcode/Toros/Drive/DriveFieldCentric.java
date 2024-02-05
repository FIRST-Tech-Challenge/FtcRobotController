package org.firstinspires.ftc.teamcode.Toros.Drive;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class DriveFieldCentric extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "FrontLeftMotor"),
                new Motor(hardwareMap, "FrontRightMotor"),
                new Motor(hardwareMap, "BackLeftMotor"),
                new Motor(hardwareMap, "BackRightMotor")
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        if(opModeIsActive()){
            while(opModeIsActive()){
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }
        }

    }


}

