package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.BlackoutRobot;
@Disabled

@Autonomous(name= "TensorTest")
public class TensorTest extends LinearOpMode {
    final boolean debug= true;
    @Override
    public void runOpMode() {
        BlackoutRobot robot = new BlackoutRobot(this, BasicChassis.ChassisType.ODOMETRY, false, false,90);
        ElapsedTime op = new ElapsedTime();

        while (!isStarted()) {
        }
        waitForStart();
    }
}
