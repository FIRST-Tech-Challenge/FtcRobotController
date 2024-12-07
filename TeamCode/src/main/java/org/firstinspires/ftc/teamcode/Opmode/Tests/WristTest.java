package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Wrist;

@TeleOp
@Config
// intake: 0.3,score: 0.95, gear keeps skipping
public class WristTest extends LinearOpMode {
    public static double angle=0.15;
    public void runOpMode(){
        Wrist wrist = new Wrist(hardwareMap);
        waitForStart();
        wrist.setPosition(0.15);
        while(opModeIsActive()&&!isStopRequested()){
            wrist.setPosition(angle);
        }

    }
}
