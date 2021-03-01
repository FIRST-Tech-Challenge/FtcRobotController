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

    public DriveCommands(UpliftTele opMode, UpliftRobot robot, DriveSubsystem driveSubsystem) {
        super(opMode, driveSubsystem);
        this.drive = driveSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        drive.robot.slowMode = false;
        double magnitude;
        double turnValue;
        // initialize the gamepad stick values to the three needed axes
        double leftY = Range.clip((-opMode.gamepad1.left_stick_y), -1, 1);
        double rightX = Range.clip(Math.pow(opMode.gamepad1.right_stick_x, 3), -1, 1);
        double leftX = Range.clip(opMode.gamepad1.left_stick_x, -1, 1);
        // Note: The following algorithm was inspired by the webpage https://seamonsters-2605.github.io/archive/mecanum/. It explains this concept very well.
        // find the angle of the left joystick
        double joystickAngle = Math.toDegrees(MathFunctions.atan2UL(leftY, leftX));

        // find the magnitude, or hypotenuse of the left joystick and scale it down by dividing by the max
        if(opMode.gamepad1.a){
            robot.slowMode = ! robot.slowMode;
        }
  
        if( robot.slowMode) {
            magnitude = ((Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)))/2);
            turnValue = ((0.7 * rightX)/2);
        }
        else {
            magnitude =  Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
            turnValue = 0.7 * rightX;
        }

        // find the turnValue directly from the rightX input value (scaled for smoothness)
        // set the powers using the 2 specific equations and clip the result
        drive.teleDrive(magnitude, joystickAngle, turnValue);

        // if dpad right is pressed on DRIVER gamepad, reset the current angle to 0
        if(opMode.gamepad1.x) {
            drive.robot.odometry.setOdometryPosition(robot.worldX, robot.worldY, 0);
        }

        // if y is pressed on DRIVER gamepad, move to the high-goal shooting position
//        if(opMode.gamepad1.y) {
//            drive.slideToPosition(DriveSubsystem.highGoalShootingPt.x, DriveSubsystem.highGoalShootingPt.y, 0.7, 0, 0.5);
//        }
//        // if x is pressed on DRIVER gamepad, move to the powershot shooting position
//        if(opMode.gamepad1.x) {
//            drive.slideToPosition(DriveSubsystem.powershotShootingPt.x, DriveSubsystem.powershotShootingPt.y, 0.7, 0, 0.5);
//        }
//        // if y is pressed on OPERATOR gamepad, set new shooting position
//        if(opMode.gamepad2.y) {
//            DriveSubsystem.highGoalShootingPt = new Point(robot.worldX, robot.worldY, robot.worldAngle);
//        }
//        // if x is pressed on OPERATOR gamepad, set new shooting position
//        if(opMode.gamepad2.x) {
//            DriveSubsystem.powershotShootingPt = new Point(robot.worldX, robot.worldY, robot.worldAngle);
//        }

    }

    @Override
    public void stop() {
        drive.stopMotors();
    }

}
