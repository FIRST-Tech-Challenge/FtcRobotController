package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonBase;
import org.firstinspires.ftc.teamcode.hardware.camera;

@Autonomous(name = "Auton Red Audience Up", group = "Autons")
public class AutonRedAudienceUp extends AutonBase {
    // Robot starts with its BACK (outtake + camera) facing the hive.
    // Intake is on the front, pointing away from the hive.
    camera cam;

    double imuSpeed = 1;
    int hiveTipped = 0;

    //where we are according to the checkTags method
    int AprilPosition = 4;

    @Override
    public void runOpMode() {
        initialize();
        cam = new camera(theHardwareMap, this);
        waitForStart();
        cam.start();

        //shoot all 4 pollen
        sleep(5000);
        //output of 1 means visible, 0 means not
        hiveTipped = cam.checkTags(AprilPosition);

        if (hiveTipped == 1) {
            encoderDrive(imuSpeed, -5, 30);
            imuTurn(imuSpeed, 90);
            imuDrive(imuSpeed, 25, 0);
            imuStrafe(imuSpeed, -6, 0, 30);
            //intake on
            sleep(500);
            imuDrive(imuSpeed, 24, 0);
            sleep(500);
            imuDrive(imuSpeed, -48, 0);
            imuTurn(imuSpeed, -90);
            //outtake on
            //shoot pollen
            sleep(3000);
            imuDrive(imuSpeed, -5, 0);
            imuTurn(imuSpeed, 135);
            imuDrive(imuSpeed, 43, 0);
            imuTurn(imuSpeed, -45);
            imuDrive(imuSpeed, 20, 0);
            //turn intake on
            sleep(2000);
            //turn intake off
            imuDrive(imuSpeed, -10, 0);
            imuStrafe(imuSpeed, 58, 0, 30);
            imuDrive(imuSpeed, 10, 0);

        }
        else {
            imuDrive(imuSpeed, -5, 0);
            imuTurn(imuSpeed, 135);
            sleep(500);
            imuDrive(imuSpeed, 38, 0);
            sleep(500);
            imuTurn(imuSpeed, -45);
            sleep(500);
            imuDrive(imuSpeed, 11, 0);
            sleep(500);
            imuStrafe(imuSpeed, 6, 0, 30);
            sleep(500);
            imuDrive(imuSpeed, 12, 0);
            //turn intake on
            sleep(2000);
            //turn intake off
            imuDrive(imuSpeed, -10, 0);
            sleep(500);
            imuStrafe(imuSpeed, 58, 0, 30);
            sleep(500);
            imuDrive(imuSpeed, 10, 0);
        }
    }
}