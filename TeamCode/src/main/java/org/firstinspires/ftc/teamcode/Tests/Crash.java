package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Warren Zhou
 * 5/23
 */
@Config
@Autonomous(name = "Crash")


public class Crash extends LinearOpMode {
    public void runOpMode() {
        //let them have second chance in case they regret selecting this program
        waitForStart();
        while (true) {
            //tell them to stop program(it won't)
            telemetry.addData("stop the program ;)", 8008);
            telemetry.update();
        }
    }
}
