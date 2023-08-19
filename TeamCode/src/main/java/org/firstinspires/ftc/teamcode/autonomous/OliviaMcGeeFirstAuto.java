package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Olivia McGee First Auto", group="Outreach")
public class OliviaMcGeeFirstAuto extends AbstractAutonomous{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        //Place your own code below here
        //The available methods are move(), turnLeft(), turnRight(), openClaw(), closeClaw()
        move(2);
        turnLeft(3);
        turnRight(2);
        openClaw();
        closeClaw();
        timer.reset();
        while (timer.seconds() > 30) {

        }
    }
}
