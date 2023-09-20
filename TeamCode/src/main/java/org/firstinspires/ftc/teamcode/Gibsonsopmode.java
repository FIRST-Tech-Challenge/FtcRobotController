package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Gibson's op mode", group = "NWR-FTC")
public class Gibsonsopmode extends LinearOpMode{
    // Definitions

    @Override
    public void runOpMode() {
        //Initialization code
        waitForStart();
        telemetry.addData("Seagulls... stop it now!!! please open README.Scam with your IP address visibly on your screen!!!, ad sponsered by yourmom.com", 0);
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        }
    }
}
