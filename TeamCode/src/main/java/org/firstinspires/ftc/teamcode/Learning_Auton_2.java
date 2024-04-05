package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Learning_Auton_2 extends LinearOpMode {
// This is a SHORT side Auton

    CyDogsChassis myBot = new CyDogsChassis(this);

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        


        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
myBot.MoveStraight(100000,0.8, 1000);
myBot.RotateLeft(1440, 0.8, 1000);

        }
    }


}


