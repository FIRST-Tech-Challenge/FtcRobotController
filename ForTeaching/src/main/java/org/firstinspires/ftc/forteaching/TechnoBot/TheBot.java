package org.firstinspires.ftc.forteaching.TechnoBot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.forteaching.TechnoBot.Subsystems.TankDriveSubsystem;

// This is the "robot" class, which is for containing all the subsystems of the bot
public class TheBot {

    // This can be convenient when working with partially completed, or mid-reconfigured robots:
    @Config
    public static class Connected {
        public static boolean DriveTrain = true;
        public static boolean Sensors = false;
    }

    // Add all our subsystems in here:
    public TankDriveSubsystem tankDriveBase;
    // public SensingSubsystem sensing;

    public TheBot(Hardware hw) {
        if (Connected.DriveTrain) {
            tankDriveBase = new TankDriveSubsystem(hw.leftDriveMotor, hw.rightDriveMotor);
        }
        if (Connected.Sensors) {
            // sensing = new SensingSubsystem(hw.colorSensor, hw.bumpSensor, hw.distanceSensor);
        }
    }
}
