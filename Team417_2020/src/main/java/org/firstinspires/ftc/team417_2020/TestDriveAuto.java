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


        move(10, 10, 0.7);
        pivot(90, 0.7);
        //move(10, 10, 0.7);
        //move(10, 0, 0.7);
        //move(0, 0, 0.7);






    }
}
