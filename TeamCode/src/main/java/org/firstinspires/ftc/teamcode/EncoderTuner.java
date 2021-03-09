package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot_utilities.GamePadController;

@Config
class DriveTrainTuner {
    public static double leftSpeed = 0.3;
    public static double rightSpeed = 0.3;
}

@TeleOp(name = "EncoderTuner")
public class EncoderTuner extends OpMode {
    DriveTrain driveTrain;
    GamePadController gamepad;

    double leftSpeed = 0;
    double rightSpeed = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                                    new Motor(hardwareMap, "dr"));
        gamepad = new GamePadController(gamepad1);
    }

    @Override
    public void loop() {
        gamepad.update();

        if(gamepad.isARelease()) {
            DriveTrainTuner.leftSpeed *= -1;
            DriveTrainTuner.rightSpeed *= -1;
        }

        if(gamepad.isYRelease()) {
            driveTrain.resetEncoders();
        }

        if(gamepad1.b) {
            leftSpeed = DriveTrainTuner.leftSpeed;
            rightSpeed = DriveTrainTuner.rightSpeed;
        } else {
            leftSpeed = 0;
            rightSpeed = 0;
        }

        driveTrain.setSpeed(leftSpeed, rightSpeed);

        int[] distances = driveTrain.getEncoderCounts();

        telemetry.addData("Left Set Speed", DriveTrainTuner.leftSpeed);
        telemetry.addData("Right Set Speed", DriveTrainTuner.rightSpeed);
        telemetry.addData("Left Distance", distances[0]);
        telemetry.addData("Right Distance", distances[1]);
        telemetry.addData("Left Distance", driveTrain.driveLeft.getCurrentPosition());
        telemetry.addData("Right Distance", driveTrain.driveRight.getCurrentPosition());
        telemetry.addData("Left Revolutions", driveTrain.driveLeft.encoder.getRevolutions());
        telemetry.addData("Right Revolutions", driveTrain.driveRight.encoder.getRevolutions());
    }
}
