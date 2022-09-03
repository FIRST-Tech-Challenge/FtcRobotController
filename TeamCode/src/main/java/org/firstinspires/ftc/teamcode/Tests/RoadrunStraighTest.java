package org.firstinspires.ftc.teamcode.Tests;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;

@Autonomous(name= "RoadrunStraighTest", preselectTeleOp = "OneGPTeleop")
public class RoadrunStraighTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ENCODER, true, false,90);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Trajectory fortyEightX = robot.roadrun.trajectoryBuilder(new Pose2d()).lineTo(new Vector2d(48,0)).build();
        waitForStart();
        robot.roadrun.followTrajectory(fortyEightX);
        while(opModeIsActive()){}
        stop();
    }
}

