package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Calvin Avila First Auto", group = "Outreach")
public class CalvinAvilaFirstAuto extends AbstractAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        //Place your own code below here
        //The available methods are move(), turnLeft(), turnRight(), openClaw(), closeClaw()
        turnLeft(1);
        turnRight(2);
        turnLeft(1);
        turnRight(2);
        turnLeft(1);
        turnRight(2);
        turnLeft(1);
        turnRight(2);
        openClaw();
        closeClaw();
        openClaw();
        closeClaw();
        move(1);
    }
}
