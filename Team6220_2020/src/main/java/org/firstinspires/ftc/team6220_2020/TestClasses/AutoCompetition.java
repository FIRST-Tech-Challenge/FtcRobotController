package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2020.MasterAutonomous;

@Autonomous(name = "Test Autonomous", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous {
    @Override
    public void runOpMode()
    {
        Initialize();

        //turnDegrees(90);

        waitForStart();


        //**this moves it 90*/
        turnToAngle(45);
        pauseMillis(1000);
        turnToAngle(35);
//
        //sleep(2000);

        //while(true){
        //    telemetry.addData("IMU:",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        //    telemetry.update();
        //}


        //turnDegrees(-270);

        //turnToAngle(75);
        ////telemetry.addData("Status : ", "Pre Setup");
        ////telemetry.update();
        ////runSetup();
//
        //telemetry.addData("RPM: ", getMotorTicksPerMinute(motorLauncher, 100));
        //telemetry.update();
        //waitForStart();
//
        ////telemetry.addData("Status : ", "12 90");
        ////telemetry.update();
        ////driveInches(60, 90);
//
        ////old 1700 millis
        //driveMillis(4000,90, 0.25);
        ////driveMillis(10,180);
        //pauseMillis(2000);
//
//
        ////driveMillis(800, 180, 0.3);
//
        //driveLauncher(1.0);
        ////driveLauncher(-1.0);
        //pauseMillis(2200);
//
        //fireLauncher(1400);
        //pauseMillis(2200);
        //driveMillis(600, 0, 0.25);
        //pauseMillis(4000);
//
        //fireLauncher(1400);
        //driveMillis(600, 0, 0.25);
        //pauseMillis(2200);
//
        //fireLauncher(1400);
        //pauseMillis(2000);
//
        //driveLauncher(0.0);
        //driveMillis(500,90, 0.5);
    }

    public void driveMillis(double millis, double direction, double power){
        driveMecanum(Math.toRadians(direction), power, 0.0);
        pauseMillis(millis);
        driveMecanum(0.0,0.0,0.0);

    }
}
