package org.firstinspires.ftc.teamcode.Configs.newConfig;

import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logLine;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This class defines the warehouse driving.
 * @author aryansinha
 * kind-of in a less prominent way
 * @soon-to-be-author karthikperi
 */
@Autonomous(name="Warehouse Drive Blue")
public class WarehouseDriveBlue extends BaseNewOpMode {
    private final HardwareNew robot;
    private final ElapsedTime runtime = new ElapsedTime();

    public WarehouseDriveBlue() {
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
     * */
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

        encoderDrive(this,1,(double)26,(double)26, 5);
        robot.getArm().setPower(0.5);
        sleep(5000);
        robot.getArm().setPower(0);
        robot.getLeftClaw().setPosition(0.5);
        robot.getRightClaw().setPosition(0.5);
        encoderDrive(this, 1, (double)-20, (double)-20, 5);
        robot.turnLeft(90, 0.2);
        encoderDrive(this,-1,(double)49,(double)49, 5);
        robot.turnRight(90, 0.2);
        encoderDrive(this, -1, 2, 2, 1);

        encoderDrive(this, 1.0, 25,25,5);
        logData(this, "Encoder Counts", String.format("Current Position is %7d :%7d",
                robot.getLeftDrive().getCurrentPosition(),
                robot.getRightDrive().getCurrentPosition()));



        robot.turnLeft(90, 0.1);
        encoderDrive(this, 1, 80, 80, 5);
        sleep(500);

    }
}











