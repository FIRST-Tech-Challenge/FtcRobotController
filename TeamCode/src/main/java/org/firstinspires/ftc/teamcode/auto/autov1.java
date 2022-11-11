package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.config.DriveUtils.*;
import static org.firstinspires.ftc.teamcode.config.DriveUtils.logData;
import static org.firstinspires.ftc.teamcode.config.DriveUtils.encoderDrive;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;

@Autonomous(name="Storage Unit")
public class autov1 extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            DriveUtils.encoderDrive(this, 0.5, 30,30, 7);
            DriveUtils.encoderDrive(this, -0.5, 10, 10, 7);
        }
    }

    public Hardware2 getRobot() {
        return robot;
    }
}