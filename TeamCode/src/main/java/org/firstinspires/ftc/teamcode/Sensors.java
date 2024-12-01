package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Sensors {
    // Total time. Never reset.
    ElapsedTime totalTime = new ElapsedTime();

    // Gyro Stabilization
    double gyroPressTime = 0;
    double gyroTarAngle;
    boolean isXReleased;

    public IMU imu;

    OpMode master;

    public void init(OpMode opMode)
    {
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        // change it to match the actual orientation of the rev control hub
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        master = opMode;
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double returnGyroYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double getTrueAngleDiff(double tarHeading)
    {
        double angle = returnGyroYaw();
        if (Math.abs(tarHeading - angle) < 180 )
        {
            return tarHeading - angle;
        }
        if (tarHeading - angle >= 180)
        {
            return  360 - tarHeading - angle;
        }
        if (angle - tarHeading <= -180)
        {
            return angle - tarHeading + 360;
        }
        return tarHeading - angle;

    }


    //////////////////////////////////////////
    public double calcAdder()
    {
        if (master.gamepad1.x)
        {
            if ((gyroPressTime < totalTime.milliseconds() - 500 || gyroPressTime == 0) && isXReleased)
            {
                gyroPressTime = totalTime.milliseconds();
                gyroTarAngle = returnGyroYaw();
            }
            double diff = getTrueAngleDiff(gyroTarAngle);
            isXReleased = false;
            double STABILIZER_CONSTANT = .05;
            return diff * STABILIZER_CONSTANT;
        }
        else
        {
            isXReleased = true;
            return 0;
        }
    }

    ///////////////////////////////////////////////////
    public void runVisionMacro(DrivetrainControllers drivetrainControllers) {

        Limelight3A limelight;
        // target class name to detect
        final String TARGET_CLASS_NAME_BLUE = "blue-face";
        final String TARGET_CLASS_NAME_RED = "red-face";
        final String TARGET_CLASS_NAME_YELLOW = "yellow-face";

        // to build a custom rumble sequence.
        Gamepad.RumbleEffect customRumbleEffect;
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                // rumble right motor 30% for 100 mSec
                .addStep(0.3, 0.3, 100)
                .build();

        // initialize Limelight and motors
        limelight = master.hardwareMap.get(Limelight3A.class, "limelight");

        // frequency that telemetry is sent to driver hub
        master.telemetry.setMsTransmissionInterval(11);

        // Switch to neural network pipeline (assuming it's pipeline 1)
        limelight.pipelineSwitch(0);

        // starts looking for data, make sure to call start() or getLatestResult() will return null.

        // initializes the limelight
        limelight.start();

        master.telemetry.addData(">", "Robot Ready.  Press Play.");
        master.telemetry.update();
        // get Limelight status and update telemetry
        LLStatus status = limelight.getStatus();
        master.telemetry.addData("LL Temp", "%.1fC", status.getTemp());
        master.telemetry.addData("LL CPU", "%.1f%%", status.getCpu());
        master.telemetry.addData("Pipeline", status.getPipelineIndex());
        // get the latest neural network result
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // access classifier results from the neural network
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();

            // check each classifier result for our target object
            boolean targetDetected = false;
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                master.telemetry.addData("Class", cr.getClassName());
                master.telemetry.addData("Confidence", "%.2f", cr.getConfidence());
                // if the target object is detected, stop the robot
                if (cr.getConfidence() > 70) {
                    if (cr.getClassName().equals(TARGET_CLASS_NAME_BLUE) || cr.getClassName().equals(TARGET_CLASS_NAME_YELLOW)) {
                        targetDetected = true;
                        drivetrainControllers.runMotorsConstantSpeed(0,0,0,0);
                        master.telemetry.addData("Status", "Target detected! Robot stopped. Rumblinnn");
                        master.gamepad1.runRumbleEffect(customRumbleEffect);
                        break;
                    }
                }
            }

            // if the target is not detected, the robot continues to strafe left
            if (!targetDetected) {
                drivetrainControllers.runMotorsConstantSpeed(-.5,.5,.5,-.5);
                master.telemetry.addData("Status", "Target not detected, strafing left...");
            }

        } else {
            master.telemetry.addData("Limelight", "No data available");
            drivetrainControllers.runMotorsConstantSpeed(-.5,.5,.5,-.5);
        }

        master.telemetry.update();
        limelight.stop();
    }
}
