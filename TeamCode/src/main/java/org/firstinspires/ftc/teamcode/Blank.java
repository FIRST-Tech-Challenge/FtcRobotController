package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp(name="Empty Opmode")
public class Blank extends LinearOpMode{
    // Definitions

    @Override
    public void runOpMode() {
        //Initialization code
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        }
    }
}
