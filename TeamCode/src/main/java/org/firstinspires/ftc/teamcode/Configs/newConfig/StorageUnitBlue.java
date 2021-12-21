package org.firstinspires.ftc.teamcode.Configs.newConfig;

import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logLine;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class defines the warehouse driving.
 * @author aryansinha
 *@soon-to-be-author karthikperi
 */
@Autonomous(name="Storage Unit")
public class StorageUnitBlue extends BaseNewOpMode {
    private final HardwareNew robot;
    private final ElapsedTime runtime = new ElapsedTime();

    public StorageUnitBlue() {
        // Start with the encoders.
        robot = new HardwareNew(true);
    }

    /**
     * {@inheritDoc}
     */
    public HardwareNew getRobot() {
        return robot;
    }

    /**
     * {@inheritDoc}
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        logLine(this, "Waiting for the robot to start");

        waitForStart();

        logLine(this,"The robot started");
        //initiating the hardware
        robot.init(hardwareMap);
        //reset Runtime
        runtime.reset();

        encoderDrive(this,.8,(double)36,(double)36, 5);
        robot.getRightClaw().setPosition(1.0);
        robot.getLeftClaw().setPosition(1.0);
        sleep(1000);
    }
}











