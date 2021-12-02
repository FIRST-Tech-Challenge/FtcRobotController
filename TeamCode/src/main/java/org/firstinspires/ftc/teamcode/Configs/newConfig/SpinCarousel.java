/*
Made by Aryan Sinha,
FTC team 202101101
 */

package org.firstinspires.ftc.teamcode.Configs.newConfig;

import static org.firstinspires.ftc.teamcode.Configs.oldConfig.selfDrive.AutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.Configs.oldConfig.selfDrive.AutoDriveUtils.logLine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configs.newConfig.HardwareNew;


/**
 * Spin Carousel
 * @author aryansinha
 */
@TeleOp(name="Spin nsjnsajdxs", group="Linear Opmode")
public class SpinCarousel extends BaseNewOpMode {
    private final HardwareNew robot = new HardwareNew(true);

    /**
     * {@inheritDoc}
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        final ElapsedTime runtime = new ElapsedTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.getCarousel().setPower(-0.8);
        }
    }
    @Override
    public HardwareNew getRobot() {
        return robot;
    }
}