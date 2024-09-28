package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "RedPark", group = "RedAutos")
public class RedPark extends LinearOpMode {
    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive);

        waitForStart();

        //Score preloaded specimen into basket.
        drivebase.autoDriveForward(0.3, 10);
        

    }
}
