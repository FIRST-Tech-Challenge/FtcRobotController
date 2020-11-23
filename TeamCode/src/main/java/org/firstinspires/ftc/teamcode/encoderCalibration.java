package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The position and target manager will not function and will throw an error if this program is not run first
 */
//todo: implement this once we get horizontal odometry, or i find a way to only use the two verticle ones
@TeleOp(name = "encoder System Calibration", group = "Calibration")
public class encoderCalibration extends LinearOpMode {
    HardwareUltimateGoal robot = new HardwareUltimateGoal("calibrating");

    final double PIVOT_SPEED = 0.5;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        robot.init(hardwareMap);

        //Encoder System Calibration Init Complete
        telemetry.addData("Encoder System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(robot.getZAngle() < 90 && opModeIsActive()){
            robot.rightDrive.setPower(PIVOT_SPEED);
            robot.leftDrive.setPower(PIVOT_SPEED);
            if(robot.getZAngle() < 60) {
                robot.leftDrive.setPower(PIVOT_SPEED);
                robot.rightDrive.setPower(PIVOT_SPEED);
            }else{
                robot.leftDrive.setPower(PIVOT_SPEED/2);
                robot.rightDrive.setPower(PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robot.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.leftDrive.getCurrentPosition()) + (Math.abs(robot.rightDrive.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*robot.NADO_COUNTS_PER_METER);

        //TODO: work around the horizontal stuff
        //horizontalTickOffset = horizontal.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        //ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            //telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.addData("Left  motor Position", -robot.leftDrive.getCurrentPosition());
            telemetry.addData("Right motor Position", robot.rightDrive.getCurrentPosition());
            //telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }
}