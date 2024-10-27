package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ExampleMecanumDriveDurationAutonomous", group = "Autonomous")
public class ExampleAutonomous extends BaseAutonomous {

    @Override
    protected void defineTasks() {
        // Add a task to drive forward for 3 seconds at 50% power
        manager.addTask(new MecanumDriveDurationTask(robot, 0.5, 0, 0, 3));
    }
}