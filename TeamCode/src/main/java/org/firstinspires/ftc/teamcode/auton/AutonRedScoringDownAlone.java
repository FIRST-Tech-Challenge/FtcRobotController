package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;

import org.firstinspires.ftc.teamcode.AutonBase;
import org.firstinspires.ftc.teamcode.hardware.camera;


@Autonomous(name = "Auton Red Scoring Down Alone", group = "Autons")
public class AutonRedScoringDownAlone extends AutonBase {
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

    }
}
