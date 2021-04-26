package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.MathFunctions;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveCommands extends Command {

    public DriveSubsystem drive;
    public UpliftTele opMode;
    UpliftRobot robot;
    boolean aPressed = false;

    public DriveCommands(UpliftTele opMode, UpliftRobot robot, DriveSubsystem driveSubsystem) {
        super(opMode, driveSubsystem);
        this.drive = driveSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        drive.robot.slowMode = false;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // slow mode toggle ON/OFF (toggles on press, holding does not do anything)
        if(opMode.gamepad1.a) {
            if (!aPressed) {
                robot.slowMode = !robot.slowMode;
                aPressed = true;
            }
        } else {
            aPressed = false;
        }

        if(opMode.gamepad1.b) {
            drive.imuStraighten();
        }

        // if x is pressed on DRIVER gamepad (when robot is up to the line and pressed against wall)
        if(opMode.gamepad1.x) {
            // set shooting state to PREPARING_POWERSHOT in order to raise transfer and set shooter vel to the powershot target vel
            robot.setShootingState(UpliftRobot.ShootingState.PREPARING_POWERSHOT);

            // set the odometry position (CHANGE/CHECK THESE VALUES LATER)
            drive.robot.odometry.setOdometryPosition(57.5, 70,0);

            // move to the first powershot shooting position
            drive.driveToPosition(DriveSubsystem.powershotShootingPt1.x, DriveSubsystem.powershotShootingPt1.y, 0.6, 0);
            drive.imuStraighten();
            robot.setShootingState(UpliftRobot.ShootingState.SHOOTING_PS1);
            while(robot.shootingState != UpliftRobot.ShootingState.DONE_PS1 && opMode.opModeIsActive()) {
                // while shooter not done with first powershot, wait
                if(!robot.safeSleep(1)) return;
            }

            // move to second powershot shooting position and set the shooter state to SHOOTING_PS2 to tell the flicker to flick the ring
            drive.driveToPosition(DriveSubsystem.powershotShootingPt2.x, DriveSubsystem.powershotShootingPt2.y, 0.6, 0);
            drive.imuStraighten();
            robot.setShootingState(UpliftRobot.ShootingState.SHOOTING_PS2);
            while(robot.shootingState != UpliftRobot.ShootingState.DONE_PS2 && opMode.opModeIsActive()) {
                // while shooter not done with second powershot, wait
                if(!robot.safeSleep(1)) return;
            }

            // move to third powershot shooting position and set the shooter state to SHOOTING_PS3 to tell the flicker to flick the ring
            drive.driveToPosition(DriveSubsystem.powershotShootingPt3.x, DriveSubsystem.powershotShootingPt3.y, 0.6, 0);
            drive.imuStraighten();
            robot.setShootingState(UpliftRobot.ShootingState.SHOOTING_PS3);
            while(robot.shootingState != UpliftRobot.ShootingState.DONE_PS3 && opMode.opModeIsActive()) {
                // while shooter not done with third powershot, wait
                if(!robot.safeSleep(1)) return;
            }

            // set the shooting state to DONE_SHOOTING in order to move the shooter to an idle speed and drop the transfer
            robot.setShootingState(UpliftRobot.ShootingState.DONE_SHOOTING);
        }

        teleOpDrive();

    }

    @Override
    public void stop() {
        drive.stopMotors();
    }

    public void teleOpDrive() {
        // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.
        double magnitude;
        double turnValue;
        // initialize the gamepad stick values to the three needed axes
        double leftY = Range.clip((-opMode.gamepad1.left_stick_y), -1, 1);
        double rightX;
//        if(opMode.gamepad1.right_stick_x < 0) {
//            rightX = -Range.clip(Math.pow(opMode.gamepad1.right_stick_x, 2), -1, 1);
//        } else {
//            rightX = Range.clip(Math.pow(opMode.gamepad1.right_stick_x, 2), -1, 1);
//        }
        rightX = Range.clip(Math.pow(opMode.gamepad1.right_stick_x, 3) / 2, -0.5, 0.5);
        double leftX = Range.clip(opMode.gamepad1.left_stick_x, -1, 1);

        // find the angle of the left joystick
        double joystickAngle = Math.toDegrees(MathFunctions.atan2UL(leftY, leftX));

        magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        turnValue = rightX;

        if(robot.slowMode) {
            magnitude /= 2;
            turnValue /= 2;
        }

        // find the turnValue directly from the rightX input value (scaled for smoothness)
        // set the powers using the 2 specific equations and clip the result
        drive.teleDrive(magnitude, joystickAngle, turnValue);
    }
}
