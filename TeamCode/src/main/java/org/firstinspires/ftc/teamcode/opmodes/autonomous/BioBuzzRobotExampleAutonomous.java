package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.broombots.sweep.Builder.SequenceBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.robot.BioBuzzRobotMovementParameters;

@Disabled
@Autonomous (group = "example")
public class BioBuzzRobotExampleAutonomous extends SWEEPRoute{
    @Override
    public void defineRoute() {
        BioBuzzRobot robot = new BioBuzzRobot(hardwareMap, telemetry);
        super.robot = robot; // override the SWEEP target robot
        sequence = new SequenceBuilder(new BioBuzzRobotMovementParameters())
                .start(0, 0, 0) // All paths must have a start point as the first
                .splineTo(10, 10, 0.5) // robot will move in a spline path toward end point from wherever the last point was (start), robot heading will follow the curve of the path
                .splineToAngle(20, 20, 90, 0.5) // robot will move in a spline path toward end point from wherever the last point was, robot heading will follow the curve of the path and end at the specified angle
                .addBreak() // adds a break in the path, robot will slow down and stop at the point, but immediately continue to the next point in the path
                .addAction(robot.exampleAction()) // adds an action to be executed at the current point in the path
                .waitAt(2) // robot will wait at the current point for the specified duration (in seconds)
                .end(60, 60, 270) // robot will move to the end point with the specified coordinates and heading
                .build(); // finalizes the path and returns a Path object

    }
}
