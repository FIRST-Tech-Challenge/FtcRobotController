package org.firstinspires.ftc.team417_2020;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Test Drive Auto")
public class TestDriveAuto extends MasterAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        autoInitializeRobot();

        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();


        pivot(90, 0.7);
        move(0, 0, 0.7);
        //x - 24 -> actually moved 21 | 2nd try - 21.5 | 3rd try - 21.5
        //y - 10 -> actually moved 9 | y - 24 -> 24
        //x - 24 (after multiplying by 0.89) -> moved around 25
        //x - 24 (after multiplying by 0.88) -> moved around 25


        //move(10, 10, 0.7);
        //move(10, 0, 0.7);
        //move(0, 0, 0.7);






    }
}
