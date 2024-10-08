package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.chassis.MegalodogChassis;



// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
public class RedRightOneSpecimen extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // this lets us see how long the op mode has run

        MegalodogChassis myBot = new MegalodogChassis(this);
        // Put code that should run during initialization HERE in this area

        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            // Put code that should run during the active mode HERE in this area

            // 1: Strafe left
            myBot.StrafeLeft(400,.3,1000);
            // 2: Move forward
            myBot.MoveStraight(550,.3,500);
            // 3: Hang specimen

            // 4: Move backwards
            myBot.MoveStraight(-470,.3,500);
            // 5: Strafe Right
            myBot.StrafeRight(800, 0.3, 1000);
            // Advanced (from where we drop specimen): Strafe right, grab first sample (closest one)
            // Rotate right, strafe left, drop sample, move backwards

        }
    }


}


