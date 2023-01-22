package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;

/**
 * @author KarthikPeri
 */

@Autonomous(name="Test")
public class test extends BaseOpMode {
    Hardware2 robot = new Hardware2(false);

    /*
    KIMCHI RAMEN
     */

    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);
        waitForStart();
        DriveUtils.encoderStrafe(this,0.4,-13.5,5);
    }
    @Override
    public Hardware2 getRobot () {
        return robot;
    }
}