package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

/**
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position
 * on the field.  The odometry algorithms will not function and will throw an error if this program is not run first.
 */
@TeleOp(name = "Odometry Auto Calibration", group = "Calibrate")
@Disabled
public class CalibrationOdometryAuto extends LinearOpMode {

    HardwarePixelbot robot = new HardwarePixelbot();

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile  = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double startHeading;    // start/current/delta in DEGREES
    double currentHeading;
    double deltaAngle;
    
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        robot.init(hardwareMap,false);

        telemetry.addData("INFO", "This program will rotate the robot 90deg");
        telemetry.addData("INFO", "clockwise and use the resulting odometer");
        telemetry.addData("INFO", "encoder counts to compute two constants");
        telemetry.addData("INFO", "specific to this robot that are saved to");
        telemetry.addData("INFO", "TXT files on the REV Control Hub for use");
        telemetry.addData("INFO", "by all other programs that depend on");
        telemetry.addData("INFO", "odometry.");
        telemetry.addData("State", "Ready");
        telemetry.update();
        waitForStart();

        // Bulk-query all odometry encoder values (BEFORE)
        // (we don't assume the encoders start at zero, so need to know BEFORE/AFTER values)
        robot.readBulkData();
        startHeading = robot.headingIMU();  // may not be ZERO at start!
        deltaAngle = 0.0;
        
        // Begin calibration
        while( opModeIsActive() && (deltaAngle < 89.0) ){
            // Set motor power
            double motorSpeed = (deltaAngle < 80.0)? 0.15 : 0.11;
            robot.driveTrainMotors( motorSpeed, -motorSpeed, motorSpeed, -motorSpeed );
            // Have we rotated the required amount yet?
            currentHeading = robot.headingIMU();
            deltaAngle = currentHeading - startHeading;
            telemetry.addData("IMU Delta Angle", deltaAngle );
            telemetry.update();
        } // opModeIsActive

        //Stop the robot (wait 2 seconds for it to fully stop)
        robot.driveTrainMotorsZero();
        sleep( 2000 );

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robot.headingIMU();

        // Bulk-query all odometry encoder values (AFTER)
        robot.readBulkData();

        int leftChange   = robot.leftOdometerCount   - robot.leftOdometerPrev;
        int rightChange  = robot.rightOdometerCount  - robot.rightOdometerPrev;
        int strafeChange = robot.strafeOdometerCount - robot.strafeOdometerPrev;

        double encoderDifference = Math.abs( leftChange ) + Math.abs( rightChange );
        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;
        double wheelBaseSeparation = ( 2 * 90 * verticalEncoderTickOffsetPerDegree ) / ( Math.PI * robot.COUNTS_PER_INCH2 );
        double horizontalTickOffset = strafeChange / Math.toRadians( angle );

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while( opModeIsActive() ) {
            telemetry.addData("State", "Odometry Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", "%.4f inches", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", "%.6f", horizontalTickOffset);
            //Display raw values
            telemetry.addData("IMU Angle", robot.headingIMU());
            telemetry.addData("Odometry left",  "%d", leftChange  );
            telemetry.addData("Odometry right", "%d", rightChange );
            telemetry.addData("Odometry rear",  "%d", strafeChange  );
            telemetry.addData("Vertical Encoder Offset", "%.6f", verticalEncoderTickOffsetPerDegree );
            //Update values
            telemetry.update();
        } // opModeIsActive()
    }

} // CalibrationOdometryAuto
