package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.autonomous.Mecanum;

@Autonomous(name = "Basic")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode(){
        Mecanum.initialize(this);

        waitForStart();

        Mecanum.driveToPosition(0.4, 150);
        Mecanum.turnToDegree(0.4, 90);
        Mecanum.driveToPosition(0.4, 250);
    }
}
