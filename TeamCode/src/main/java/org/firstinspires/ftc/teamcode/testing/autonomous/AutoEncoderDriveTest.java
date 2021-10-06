package org.firstinspires.ftc.teamcode.testing.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.motors.HDrive;

@Autonomous(name="Encoder Drive Test (H-Drive)", group="H.Testing.Autonomous")
public class AutoEncoderDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        MotorController motorController = new MotorController(telemetry, hardwareMap, this);
//        motorController.drive(MotorController.DRIVE_SPEED, 12, 12, 0, 4.0, "First");
        HDrive drivetrain = new HDrive(telemetry, hardwareMap);
        drivetrain.move(12, 1, 10);
    }

}
