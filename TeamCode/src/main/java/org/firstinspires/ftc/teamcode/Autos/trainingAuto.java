package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.SignificantMotionDetection;
import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "trainingAuto")
public class trainingAuto extends LinearOpMode {
    private double SIDE_SPEED = .2;

    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        //Put code here
        drivebase.driveSideways(SIDE_SPEED, -8, 0);
    }
}
