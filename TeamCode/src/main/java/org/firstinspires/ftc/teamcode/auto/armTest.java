package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.BaseOpMode;
import org.firstinspires.ftc.teamcode.config.DriveUtils;
import org.firstinspires.ftc.teamcode.config.Hardware2;

/**
 * @author KarthikPeri
 */
@Autonomous(name="armTest")
public class armTest extends BaseOpMode {
    Hardware2 robot = new Hardware2(true);

    /*
    KIMCHI RAMEN
     */

    public void runOpMode() throws InterruptedException {
        robot.initTeleOpIMU(hardwareMap);
        waitForStart();

        robot.getArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveUtils.encoderClaw(this, -0.6,1825,5);
        //robot.getArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveUtils.encoderClaw(this,0.5,-1825,10);

        }
    @Override
    public Hardware2 getRobot () {
        return robot;
    }
}