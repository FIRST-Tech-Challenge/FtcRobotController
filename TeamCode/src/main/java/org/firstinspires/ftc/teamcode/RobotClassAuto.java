package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.libraries.AutoRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//import org.firstinspires.ftc.teamcode.auto.BasicRobot;


@Autonomous
public class RobotClassAuto extends LinearOpMode /*implements BasicRobot*/ {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry);
        waitForStart();

        robot.getImu().resetYaw(); //if you move the robot at all between init and running
        //yaw will be incorrect, this handles that
        telemetry.addData("Status", "Running");
        telemetry.update();
        // elevator(3300);
        robot.driveBackwardsSeconds(.2);
        robot.face(10);
        robot.driveArcLeftForwards(80);





    }
}