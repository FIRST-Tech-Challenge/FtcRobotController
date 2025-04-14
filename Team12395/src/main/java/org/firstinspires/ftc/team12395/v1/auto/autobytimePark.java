package org.firstinspires.ftc.team12395.v1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.v1.RobotHardware;


@Autonomous(name = "Robot: Auto By time Park", group = "Robot")

public class autobytimePark extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    public ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;

    double drive = 0;
    double strafe  = 0;
    double turn = 0;

    @Override
    public void runOpMode(){

        robot.init();
        waitForStart();

        robot.driveRobotCentric(0.4, 0, 0);
        sleep(2100);

        robot.driveRobotCentric(0, 0.4, 0 );
        sleep(600);

        robot.driveRobotCentric(0, 0, 0);
    }

}
