package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumDriveDurationTask extends AutonomousTask {
    private double forward; // Forward/backward power level
    private double strafe; // Left/right strafing power level
    private double rotate; // Rotation power level
    private double time; // Duration for the drive
    private ElapsedTime timer; // Timer to track task duration

    // Constructor to initialize the mecanum drive task with power levels and time
    public MecanumDriveDurationTask(Robot robot, double forward, double strafe, double rotate, double time) {
        super(robot);
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.time = time;
        this.timer = new ElapsedTime(); // Initialize and reset timer
    }

    @Override
    public void execute() {
        // Drive using mecanum drive if the task is not complete, otherwise stop
        if (!isComplete()) {
            robot.drive(forward, strafe, rotate);
        } else {
            robot.drive(0, 0, 0); // Stop the robot
        }
    }

    @Override
    public boolean isComplete() {
        // Task is complete if the elapsed time is greater than or equal to the specified time
        return timer.seconds() >= time;
    }
}