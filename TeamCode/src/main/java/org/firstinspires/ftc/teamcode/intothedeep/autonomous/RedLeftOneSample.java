package org.firstinspires.ftc.teamcode.intothedeep.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.core.CyDogsChassis;


// The 'extends LinearOpMode' is needed so this code can run the build in op mode code from FIRST.
//    @Autonomous puts this code in the autonomous category on driver station
@Autonomous
public class RedLeftOneSample extends LinearOpMode {

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

            // 1: Drive forward
            myBot.MoveStraight(75,.3,400);
            // 2: Rotate left 90 degrees
            myBot.RotateRight(48,.3,400);
            // 3: Drive forward
            myBot.MoveStraight(690,.3,400);
            // 4: Rotate left 45 degrees
            myBot.RotateRight(25,.3,400);
            // 5: Drop sample

            // 6: Rotate right 45 degrees
            myBot.RotateLeft(25,.3,400);
            // 7: Drive backwards
            myBot.MoveStraight(-670,.3,400);
            // 8: Strafe right to low chamber
            myBot.StrafeRight(920,.3,400);
            // 9: Rotate 90 degrees right
            myBot.RotateLeft(70,.3,400);
            // 10: Touch low chamber close to left side
            myBot.MoveStraight(100,.3,400);
            // pts:11 w/ mechanisms

        }
    }


}


