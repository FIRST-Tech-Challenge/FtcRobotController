package org.firstinspires.ftc.team00000.v1.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.v1.RobotHardware;
@Disabled
@Autonomous(name = "Neutral Samples", group = "Autonomous")

public class AutoDriveByEncoder_Neutral extends LinearOpMode {

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
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_BASKET);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_BUCKET);
        robot.driveOmni(robot.DRIVE_SPEED, 42, 195, 0);
        robot.turnToHeading(robot.TURN_SPEED, 45);
        robot.setVerticalClawPosition(robot.CLAW_OPEN);
        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
        robot.turnToHeading(robot.TURN_SPEED, 90);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.driveOmni(robot.DRIVE_SPEED, 11, 65, 0);
        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
        sleep(750);
        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
        sleep(250);
        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
        robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_BASKET);
        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_BUCKET);
        robot.turnToHeading(robot.TURN_SPEED, 45);
        robot.driveOmni(robot.DRIVE_SPEED, 10, 170, 45);

//        robot.driveStraight(robot.DRIVE_SPEED, -10, 45);
//        robot.setVerticalClawPosition(robot.CLAW_OPEN);
//
//        sleep(250);
//
//        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
//        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
//        robot.turnToHeading(robot.TURN_SPEED, 90);
//        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
//        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
//        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
//        robot.driveLateral(robot.STRAFE_SPEED, -3.5, 90);
//        robot.driveAxial(robot.DRIVE_SPEED, 7,90);
//        robot.driveAxial(0.2, 1, -90);
//        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
//
//        sleep(250);
//
//        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
//        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
//
//        sleep(750);
//
//        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
//
//        sleep(250);
//
//        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
//        robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_BASKET);
//        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_BUCKET);
//        robot.driveStraight(robot.DRIVE_SPEED, -3, 45);
//        robot.turnToHeading(robot.TURN_SPEED, 45);
//        robot.setVerticalClawPosition(robot.CLAW_OPEN);
//
//        sleep(250);
//
//        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
//        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);
//        robot.turnToHeading(robot.TURN_SPEED, 90);
//        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MAX);
//        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_PICKUP);
//        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
//
//        sleep(250);
//
//        robot.driveLateral(robot.STRAFE_SPEED, 4, 90);
//        robot.turnToHeading(robot.TURN_SPEED,118);
//        robot.driveAxial(robot.DRIVE_SPEED, 4, 118);
//        robot.driveAxial(0.2, 1, 118);
//        robot.turnToHeading(0.2, 113);
//        robot.turnToHeading(robot.TURN_SPEED, 103);
//        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
//        robot.turnToHeading(robot.TURN_SPEED, 118);
//        robot.driveAxial(0.2, 2, 118);
//        robot.setHorizontalClawPosition(robot.CLAW_CLOSE);
//
//        sleep(250);
//
//        robot.setHorizontalArmPosition(robot.HORIZONTAL_ARM_MIN);
//        robot.setHorizontalWristPosition(robot.HORIZONTAL_WRIST_TRANSFER);
//
//        sleep(750);
//
//        robot.setVerticalClawPosition(robot.CLAW_CLOSE);
//
//        sleep(250);
//
//        robot.setHorizontalClawPosition(robot.CLAW_OPEN);
//        robot.setVerticalArmPosition(robot.VERTICAL_ARM_HIGH_BASKET);
//        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_BUCKET);
//        robot.turnToHeading(robot.TURN_SPEED, 62);
//        robot.driveAxial(robot.DRIVE_SPEED, -6, 62);
//        robot.setVerticalClawPosition(robot.CLAW_OPEN);
//
//        sleep(250);
//
//        robot.setVerticalArmPosition(robot.VERTICAL_ARM_MIN);
//        robot.setVerticalWristPosition(robot.VERTICAL_WRIST_TRANSFER);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pauses to display the final telemetry message.
    }
}
