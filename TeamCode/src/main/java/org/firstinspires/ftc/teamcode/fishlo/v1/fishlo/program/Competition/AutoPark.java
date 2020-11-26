package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class AutoPark extends FishloAutonomousProgram {

    //Builds the robot
    @Override
    public Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    //This method is for code that needs to run during the init phase.
    @Override
    public void preMain() {
        gyro.initGyro();
    }

    //This method is for code that needs to run during the init phase
    @Override
    public void main() {
        //Status update: The robot is strafing to the middle of the line
        telemetry.addData("Main", "Strafing - P: -40, S: 0.4");
        telemetry.update();
        //Strafes to the middle of the line
        drive.strafeToPosition(-40, 0.4);
        //Status update: The robot is driving to the line
        telemetry.addData("Main", "Driving - P: 70, S: 0.5");
        telemetry.update();
        //Drives to the line
        drive.moveToPosition(70, 0.5);
    }
}
