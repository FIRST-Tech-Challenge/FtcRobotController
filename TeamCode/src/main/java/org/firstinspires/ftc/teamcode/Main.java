package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Its just like the "Pose2d" class but ours, so that we can make changes and additions in the class easily.
import org.firstinspires.ftc.teamcode.CustomClasses.Position;


@Autonomous(name = "Main")
public class Main extends LinearOpMode {
    //Position s = new Position();
    
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {

            }
        }
    }
}
