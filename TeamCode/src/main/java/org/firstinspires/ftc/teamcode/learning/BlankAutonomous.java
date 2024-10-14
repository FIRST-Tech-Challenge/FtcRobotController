package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
public class BlankAutonomous extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // Put code that should run during initialization HERE in this area


        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            // Put code that should run during the active mode HERE in this area

        }
    }


}



