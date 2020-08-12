package org.firstinspires.ftc.teamcode.rework.Robot.Modules.Odometry;

import org.firstinspires.ftc.teamcode.rework.Robot.Modules.Module;
import org.firstinspires.ftc.teamcode.rework.Robot.ReworkRobot;

/**
 * Odometry includes all everything required to calculate the robot's position throughout
 * TeleOp or Autonomous.
 */
public class Odometry extends Module {
    RobotPosition robotPosition;

    ReworkRobot robot;

    // Constants (easily modified for changing hardware)
    private static final int LEFT_POD_ENCODER_PORT = 0;
    private static final int RIGHT_POD_ENCODER_PORT = 1;
    private static final int MECANUM_POD_ENCODER_PORT = 2;

    public Odometry(ReworkRobot robot) {
        this.robot = robot; // Odometry needs robot in order to be able to get data from robot
        // Odometry is the only module that won't need the hardwareMap, as it doesn't move anything

        init();
    }

    public void init() {}

    public synchronized void update() {
        calculateRobotPosition();
    }

    public synchronized RobotPosition getRobotPosition() {
        return robotPosition;
    }

    double leftPodOldPosition = 0;
    double rightPodOldPosition = 0;
    double mecanumPodOldPosition = 0;
    /**
     * Calculates the robot's position.
     */
    private void calculateRobotPosition() {
        double leftPodNewPosition = robot.getRevHub1Data().getMotorCurrentPosition(LEFT_POD_ENCODER_PORT);
        double rightPodNewPosition = robot.getRevHub1Data().getMotorCurrentPosition(RIGHT_POD_ENCODER_PORT);
        double mecanumPodNewPosition = robot.getRevHub1Data().getMotorCurrentPosition(MECANUM_POD_ENCODER_PORT);

        double leftPodDelta = leftPodNewPosition - leftPodOldPosition;
        double rightPodDelta = rightPodNewPosition - rightPodOldPosition;
        double mecanumPodDelta = mecanumPodNewPosition - mecanumPodOldPosition;

        circularOdometry(leftPodDelta, rightPodDelta, mecanumPodDelta);

        leftPodOldPosition = leftPodNewPosition;
        rightPodOldPosition = rightPodNewPosition;
        mecanumPodOldPosition = mecanumPodNewPosition;
    }

    private void circularOdometry(double leftPodDelta, double rightPodDelta, double mecanumPodDelta) {
        // TODO circularOdometry()
    }
}
