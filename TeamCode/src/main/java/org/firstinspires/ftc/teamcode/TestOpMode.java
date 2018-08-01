package org.firstinspires.ftc.teamcode;

import com.acmerobotics.splinelib.trajectory.AssetsTrajectoryLoader;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

@Autonomous
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Trajectory trajectory = AssetsTrajectoryLoader.load("test");
            System.out.println(trajectory.duration());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
