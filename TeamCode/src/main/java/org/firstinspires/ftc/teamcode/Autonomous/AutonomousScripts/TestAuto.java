package org.firstinspires.ftc.teamcode.Autonomous.AutonomousScripts;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Path;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Waypoint;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

@Config
@Autonomous(name = "TEST AUTO")
public class TestAuto extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    HardwareMap hardwareMap;
    AutonomousRobot autoRobot;
    Drivetrain drivetrain;

    Path p = new Path.PathBuilder()
                .addNewFullPoint(
                        new Waypoint(1, 1, 90, 75, 1, DistanceUnit.INCH),
                        () -> {},
                        1)
                        .build();


    @Override
    public void runOpMode() throws InterruptedException {
        autoRobot = new AutonomousRobot(hardwareMap);

        drivetrain.startOdometry();
        //autoRobot.setPath(p);
        autoRobot.startPath();

        waitForStart();

        timer.reset();

        while(opModeIsActive()){
            autoRobot.step();
        }
    }

}
