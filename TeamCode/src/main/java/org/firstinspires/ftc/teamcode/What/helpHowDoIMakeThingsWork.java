package org.firstinspires.ftc.teamcode.What;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Motors;

public class helpHowDoIMakeThingsWork extends LinearOpMode {

    Motors notServos;

    @Override
    public void runOpMode() throws InterruptedException {

        notServos = new Motors(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            notServos.MoveMotor(0, 0.5);
            notServos.MoveMotor(1, 0.5);
            notServos.MoveMotor(2, 0.5);
            notServos.MoveMotor(3, 0.5);
            sleep(500);
            notServos.MoveMotor(0,0);
            notServos.MoveMotor(1,0);
            notServos.MoveMotor(2,0);
            notServos.MoveMotor(3,0);
            sleep(500);
        }
    }
}