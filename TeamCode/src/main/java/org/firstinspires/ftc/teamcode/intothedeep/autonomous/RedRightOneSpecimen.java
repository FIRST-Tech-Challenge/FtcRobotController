package org.firstinspires.ftc.teamcode.intothedeep.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.core.CyDogsChassis;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
public class RedRightOneSpecimen extends LinearOpMode {

    /* declare variables

     */
    @Override
    public void runOpMode() {

        // this lets us see how long the op mode has run

        CyDogsChassis myBot = new CyDogsChassis(this);
        // Put code that should run during initialization HERE in this area
        myBot.BackLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        myBot.FrontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        myBot.FrontRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        myBot.BackRightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // Wait for the start button to be pressed on the driver station
        waitForStart();

        if (opModeIsActive()) {
            // Put code that should run during the active mode HERE in this area

            // 1: Drive forward, might need to move a little bit backwards
            myBot.MoveStraight(590,.3,400);
            // 2: Strafe left a little

            // 3: Hang specimen

            // 4: Touch chamber

            // 5:

            // 6:



        }
    }


}


