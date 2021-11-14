package org.firstinspires.ftc.teamcode.selfDrive;

import static org.firstinspires.ftc.teamcode.selfDrive.AutoDriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.selfDrive.AutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.selfDrive.AutoDriveUtils.logLine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Hardware2;

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
    @Override
    public Hardware2 getRobot() {
        return robot;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        logLine(this, "Waiting for the robot to start");

        waitForStart();

        logLine(this,"The robot started");
        //initiating the hardware
        robot.init(hardwareMap);
        //reset Runtime
        runtime.reset();

        //while (opModeIsActive()) { //TODO: add the time.
        encoderDrive(this, 1.0, 12,12,5);
        logData(this, "Encoder Counts", String.format("Current Position is %7d :%7d",
                robot.getLeftDrive().getCurrentPosition(),
                robot.getRightDrive().getCurrentPosition()));
        //}
    }
}











