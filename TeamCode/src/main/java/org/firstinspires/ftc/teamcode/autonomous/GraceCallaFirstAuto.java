package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="Grace Calla First Auto", group = "Outreach")
public class GraceCallaFirstAuto extends AbstractAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        //Place your own code below here
        //The available methods are move(), turnLeft(), turnRight(), openClaw(), closeClaw()
        move(4);
        turnLeft(3);
        move(6);
        openClaw();
        closeClaw();
        move(4);
    }
}
