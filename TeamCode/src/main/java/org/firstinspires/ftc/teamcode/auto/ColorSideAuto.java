package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ColorSideAuto")
public class ColorSideAuto extends LinearOpMode {

    //State Machine Variables
    enum States {
        DEPOSIT_SAMPLE, HANG_SPECIMIN,
        COLLECT_SAMPLE, COLLECT_SPECIMIN,
        START_TO_SPECIMIN_HANG,
        SPECIMIN_HANG_TO_SPIKE,
        SPIKE_TO_HUMAN,
        HUMAN_TO_NEXT_SPIKE

    };

    States currentState;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init robot
        currentState = States.START_TO_SPECIMIN_HANG;
        waitForStart();
        //Auto Begins

    }
}
