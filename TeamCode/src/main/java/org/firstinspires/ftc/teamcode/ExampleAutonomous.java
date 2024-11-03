package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ExampleMecanumDriveDistanceAutonomous", group = "Autonomous")
public class ExampleAutonomous extends BaseAutonomous {
    @Override
    protected void defineTasks() {
        // Add a task to drive forward for 1 meter at 50% power
        manager.addTask(new MecanumDriveDistanceTask(robot, 1.0, this));
    }
}