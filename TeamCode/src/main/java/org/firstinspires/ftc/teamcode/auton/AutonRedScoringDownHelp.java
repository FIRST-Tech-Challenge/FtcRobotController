package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;

import org.firstinspires.ftc.teamcode.AutonBase;
import org.firstinspires.ftc.teamcode.hardware.camera;


@Autonomous(name = "Auton Red Scoring Down Help", group = "Autons")
public class AutonRedScoringDownHelp extends AutonBase {
    // Robot starts with back against center of blue depot
    camera cam;

    double imuSpeed = 1;

    int hiveTipped = 0;

    //where we are according to the checkTags method
    int AprilPosition = 2;

    @Override
    public void runOpMode() {
        initialize();
        cam = new camera(theHardwareMap, this);
        waitForStart();
        cam.start();


        //wait to see if other team tipped hive
        encoderDrive(imuSpeed, 5, 0);

        sleep(10000);

        //output of 1 means visible, 0 means not
        hiveTipped = cam.checkTags(AprilPosition);

        //if we can see it, success
        if (hiveTipped == 1) {
            //shoot 4 preloaded pollen
            sleep(5000);
            imuTurn(imuSpeed, -90);
            imuDrive(imuSpeed, 6, 0);
            //turn on intake
            sleep(1000);
            //turn off intake
            sleep(1000);
            //"turn towards" according to strategy team
            imuTurn(imuSpeed, 90);
            //shoot 4 pollen
            sleep(5000);
            //go park
            imuDrive(imuSpeed, -12, 0);
            imuStrafe(imuSpeed, -63, 0, 30);
        }
        else {
            imuDrive(imuSpeed, -100, 0);
            sleep(500);
            imuStrafe(imuSpeed, -35, 0, 30);
            sleep(500);
            imuDrive(imuSpeed, -15, 0);
            imuTurn(imuSpeed, -135);
            //shoot 4 preloaded pollen
            sleep(5000);
            imuTurn(imuSpeed, 135);
            sleep(500);
            encoderStrafe(imuSpeed, -20 , 30);
            sleep(500);
            imuDrive(imuSpeed, 70, 0);
            imuStrafe(imuSpeed, -7, 0, 30);
        }
    }
}
