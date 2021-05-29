package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.driver.Driver;
import org.firstinspires.ftc.teamcode.driver.HDriveMode;
import org.firstinspires.ftc.teamcode.driver.HDriver;
import org.firstinspires.ftc.teamcode.driver.StandardDriveMode;
import org.firstinspires.ftc.teamcode.driver.StandardDriver;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPartSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.DriveTrainSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.HDrive;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.StandardDrive;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;
import org.firstinspires.ftc.teamcode.buttons.AllMappings;

import java.util.ArrayList;

/**
 * Main drive to autodetect what drive and set it to be omni wheels
 * @author 22jmiller
 */
@TeleOp(name="Auto-Detect Drive Omni", group="Drive")
public class MainDrive extends OpMode {
    Driver driver;

    Robot robot;

    AllMappings mappings;

    @Override
    public void init() {
        // Robot parts
        ArrayList<RobotPartSettings> robotPartSettings = new ArrayList<RobotPartSettings>();

        DriveTrainSettings driveTrainSettings = new DriveTrainSettings(true, WheelTypes.OMNI);
        driveTrainSettings.driveWithEncoder = true;
        robotPartSettings.add(driveTrainSettings);

        // Robot
        this.robot = new Robot(true, robotPartSettings,gamepad1,gamepad2,telemetry,hardwareMap);

        this.mappings = new AllMappings(this.robot);

        // Pick driver from class
        if (robot.hasRobotPart(HDrive.class)) {
            telemetry.addData("Using HDrive", true);
            driver = new HDriver(true, HDriveMode.MINECRAFT, robot);
        } else {
            telemetry.addData("Using HDrive", false);
            driver = new StandardDriver(true, StandardDriveMode.GTA, robot);
        }

    }

    @Override
    public void start() {
        driver.start();
        this.mappings.start();
    }

    @Override
    public void stop() {
        driver.stop();
        this.mappings.stop();
    }

    @Override
    public void loop() {
        driver.loop();
    }
}
