package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

class MecanumDriveDurationTask extends AutonomousTask {
    private double forward; // Forward/backward power level
    private double strafe; // Left/right strafing power level
    private double rotate; // Rotation power level
    private double time; // Duration for the drive
    private ElapsedTime timer; // Timer to track task duration
    private LinearOpMode opMode; // Reference to opMode for telemetry

    // Constructor to initialize the mecanum drive task with power levels and time
    public MecanumDriveDurationTask(Robot robot, double forward, double strafe, double rotate, double time, LinearOpMode opMode) {
        super(robot);
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.time = time;
        this.timer = new ElapsedTime(); // Initialize timer
        this.opMode = opMode;
        System.out.println(String.format("MecanumDriveDurationTask initialized - Forward: %2.2f, Strafe: %2.2f, Rotate: %2.2f, Time: %2.2f", forward, strafe, rotate, time));
    }

    @Override
    public void execute() {
        // Reset timer immediately before starting the task to avoid delay issues
        if (timer.seconds() == 0) {
            timer.reset();
        }
        // Continuously add telemetry data for monitoring
        if (!isComplete()) {
            opMode.telemetry.addData("Time elapsed", String.format("%2.2f / %2.2f seconds", timer.seconds(), time));
            opMode.telemetry.update();
            robot.drive(forward, strafe, rotate);
        } else {
            // Stop the robot once the task is complete
            robot.drive(0, 0, 0);
            opMode.telemetry.addData("Task Status", "MecanumDriveDurationTask complete - Robot stopped");
            opMode.telemetry.update();
            System.out.println("MecanumDriveDurationTask complete - Robot stopped");
        }

    }

    @Override
    public boolean isComplete() {
        // Ensure motors are stopped if time is complete
        if (timer.seconds() >= time) {
            robot.drive(0, 0, 0);
        }
        // Task is complete if the elapsed time is greater than or equal to the specified time
        boolean complete = timer.seconds() >= time;
        if (complete) {
            System.out.println("MecanumDriveDurationTask is complete");
        }
        return complete;
    }
}