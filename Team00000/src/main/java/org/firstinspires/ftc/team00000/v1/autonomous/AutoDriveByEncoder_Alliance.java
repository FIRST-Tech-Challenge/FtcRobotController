package org.firstinspires.ftc.team00000.v1.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.v1.RobotHardware;
@Disabled
@Autonomous(name = "Alliance Samples", group = "Autonomous")

public class AutoDriveByEncoder_Alliance extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){
        double horizontalArm;

        // Initialize all the hardware using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_RUNG - robot.WRIST_FUDGE);
        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.driveOmni(robot.DRIVE_SPEED, 28, 180, 0);
        robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_RUNG);
        sleep(750);
        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
        robot.setVerticalClawPosition(robot.CLAW_OPEN);
        robot.driveOmni(robot.DRIVE_SPEED, 47, -Math.toDegrees(Math.atan(45.0/15.0)), 0);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.turnToHeading(robot.TURN_SPEED, 180);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
        sleep(500);
        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
        sleep(750);
        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_DROP);
        robot.driveOmni(robot.STRAFE_SPEED, 12, 90, 180);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
        robot.setVerticalClawPosition(robot.CLAW_OPEN);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
        sleep(500);
        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_WALL);
        robot.turnToHeading(robot.TURN_SPEED, -0);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.driveOmni(robot.DRIVE_SPEED, 25, 90, 0);
        robot.turnToHeading(robot.TURN_SPEED,0);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
        sleep(750);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_WALL + 0.04);
        sleep(250);
        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
        sleep(750);
        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.driveOmni(robot.DRIVE_SPEED, 38, 90 + Math.toDegrees(Math.atan(30.0/50.0)), 0);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_RUNG - robot.WRIST_FUDGE);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pauses to display the final telemetry message.
    }
}
