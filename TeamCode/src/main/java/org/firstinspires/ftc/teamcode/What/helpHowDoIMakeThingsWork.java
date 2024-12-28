package org.firstinspires.ftc.teamcode.What;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Motors;

@Autonomous (name = "Sam motor")
public class helpHowDoIMakeThingsWork extends LinearOpMode {

    Motors motors;

    @Override
    public void runOpMode() throws InterruptedException {

        motors = new Motors(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            motors.MoveMotor(0, 50);
            motors.MoveMotor(1, 50);
            motors.MoveMotor(2, 50);
            motors.MoveMotor(3, 50);
            sleep(500);
            motors.MoveMotor(0,0);
            motors.MoveMotor(1,0);
            motors.MoveMotor(2,0);
            motors.MoveMotor(3,0);
            sleep(500);
        }
    }
}