package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

class MecanumDriveDistanceTask extends AutonomousTask {
    private static final double WHEEL_DIAMETER_METERS = 0.096; // 96 mm diameter for GoBilda 5203 Yellow Jacket
    private static final double TICKS_PER_REVOLUTION = 537.6; // GoBilda 5203 series motor
    private static final double CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    private double distanceMeters; // Distance to move in meters
    private int targetTicks; // Target encoder ticks
    private LinearOpMode opMode;

    public MecanumDriveDistanceTask(Robot robot, double distanceMeters, LinearOpMode opMode) {
        super(robot);
        this.distanceMeters = distanceMeters;
        this.targetTicks = (int) ((distanceMeters / CIRCUMFERENCE_METERS) * TICKS_PER_REVOLUTION);
        this.opMode = opMode;

        // Configure the motors to run to a specific target position
        robot.getDriveSystem().resetAndRunToPosition();
        robot.getDriveSystem().setTargetPositionAndRun(targetTicks);
        System.out.println(String.format("MecanumDriveDistanceTask initialized to move %.2f meters (%d ticks)", distanceMeters, targetTicks));
    }

    @Override
    public void execute() {
        // Set motor power to move to the target position
        robot.getDriveSystem().setTargetPositionAndRun(targetTicks); // Set target position and run motors accordingly

        // Display telemetry
        opMode.telemetry.addData("Target Position", targetTicks);
        opMode.telemetry.addData("Current Position", robot.getDriveSystem().getFrontLeftPosition());
        opMode.telemetry.update();
    }

    @Override
    public boolean isComplete() {
        // The task is complete when all motors have reached the target position
        boolean complete = !robot.getDriveSystem().isBusy();
        if (complete) {
            robot.getDriveSystem().resetAndRunToPosition(); // Stop all motors by resetting to default state
            System.out.println("MecanumDriveDistanceTask is complete - Robot stopped.");
        }
        return complete;
    }
}