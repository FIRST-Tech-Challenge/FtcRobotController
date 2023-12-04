package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

public class BlueBackstage extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
        }

        //move forwards to the wall

    }
}
