package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;

/**
 * @author KarthikPeri
 */
@Disabled
@Autonomous(name="autov1")
public class autov1 extends BaseOpMode {
    Hardware2 robot = new Hardware2(false);

    /*
    KIMCHI RAMEN
     */

    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);
        int position = -1;
        waitForStart();

        if (position == 0) {
            DriveUtils.encoderDrive(this, 0.5, -24, -24, 7);
            // Drives forward to go into the middle parking area
            //DriveUtils.encoderDrive(this, 0.5, 10, 10, 7);
        } else if (position == 1) {
            // Move Left
            DriveUtils.encoderStrafe(this,0.4,27,5);
            // Strafes left
            DriveUtils.encoderDrive(this, 0.4, -12, -12, 5);
            // Drives forward to go into the left parking area
        } else if (position == -1) {
            // Move Right
            DriveUtils.encoderStrafe(this, 0.4, -27, 5);
            //Strafes Right
            DriveUtils.encoderDrive(this, 0.4, -12, -12, 5 );
            // Drives forward to go into the left parking area
        }

        }
    @Override
    public Hardware2 getRobot () {
        return robot;
    }
}