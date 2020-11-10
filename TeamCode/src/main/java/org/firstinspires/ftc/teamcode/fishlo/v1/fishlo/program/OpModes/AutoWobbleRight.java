package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.OpModes;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoWobbleRight extends FishloAutonomousProgram {

    //Build the robot
    @Override
    public Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void main() {
        //Initialize the imu
        gyro.initGyro();
        //Find the target zone based on the starter stack
        char targetZone = vision.getTargetZone();
        //Print the target zone found on screen
        telemetry.addData("Target Zone", targetZone);

        //Wait for Start button to be pressed
        waitForStart();

        //Move wobble goal to target zone A
        if (targetZone == 'A') {
            drive.moveToPosition(55, 0.5);
            sleep(50);
            //Insert code to Turn RIGHT 90 degrees with 0.2 power
            sleep(50);
            drive.moveToPosition(-20, 0.5);
            sleep(50);
            drive.strafeToPosition(-9, 0.3);
            sleep(50);
        }
        //Move wobble goal to target zone B
        else if (targetZone == 'B') {
            //Insert code to Turn LEFT 5 degrees at 0.2 power
            drive.moveToPosition(80,0.5);
            sleep(50);
            drive.strafeToPosition(-10,0.4);
            sleep(50);
            drive.moveToPosition(-15,0.4);
        }
        //Move wobble goal to target zone C
        else if (targetZone == 'C') {
            //Insert code to Turn RIGHT 5 degrees at 0.2 power
            drive.moveToPosition(100,0.5);
            sleep(50);
            drive.moveToPosition(-35,0.5);
            sleep(50);
            drive.moveToPosition(-18,0.5);
        }
    }
}
