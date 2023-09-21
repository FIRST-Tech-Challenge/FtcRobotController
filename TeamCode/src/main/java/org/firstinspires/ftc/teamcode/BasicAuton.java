package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// All this auton does is just move forward using dead reckoning
@Autonomous
public class BasicAuton extends LinearOpMode {
    BasicRobot robot = new BasicRobot(this);
    // constructor
    public BasicAuton() throws InterruptedException {
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // dummy
    }
}
