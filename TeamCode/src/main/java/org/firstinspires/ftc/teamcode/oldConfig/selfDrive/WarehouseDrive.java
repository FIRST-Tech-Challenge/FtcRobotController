package org.firstinspires.ftc.teamcode.oldConfig.selfDrive;

import static org.firstinspires.ftc.teamcode.oldConfig.selfDrive.AutoDriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.oldConfig.selfDrive.AutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.oldConfig.selfDrive.AutoDriveUtils.logLine;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oldConfig.BaseOpMode;
import org.firstinspires.ftc.teamcode.oldConfig.Hardware2;

/**
 * This class defines the warehouse driving.
 * @author aryansinha
 */
@Autonomous(name="Warehouse Driving")
public class WarehouseDrive extends BaseOpMode {
    private final Hardware2 robot;
    private final ElapsedTime runtime = new ElapsedTime();

    public WarehouseDrive() {
        // Start with the encoders.
        robot = new Hardware2(true);
    }

    /**
     * {@inheritDoc}
     */
    public Hardware2 getRobot() {
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

        encoderDrive(this,1,(double)49,(double)49, 5);
        robot.turnLeft(90, 0.2);
        encoderDrive(this, 1, 2, 2, 1);
        robot.getClawServo().setPosition(0.1);
        encoderDrive(this,-1,(double)49,(double)49, 5);
        robot.turnRight(90, 0.2);
        encoderDrive(this, -1, 2, 2, 1);
        robot.getClawServo().setPosition(0.1);
        encoderDrive(this, 1.0, 25,25,5);
        logData(this, "Encoder Counts", String.format("Current Position is %7d :%7d",
                robot.getLeftDrive().getCurrentPosition(),
                robot.getRightDrive().getCurrentPosition()));


        robot.turnLeft(90, 0.1);
        encoderDrive(this, 1, 80, 80, 5);
        robot.getClaw().setPower(1);
        sleep(500);
        robot.getClaw().setPower(0);
        robot.getClawServo().setPosition(1);
        robot.getClawServo().setPosition(0);


    }
}











