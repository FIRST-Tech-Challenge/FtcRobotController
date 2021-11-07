package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.BlueVisionRGBNoTele;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Autonomous
public class CompBotV3 {
    public final static double distanceK = 384.5/(100*Math.PI)*25.4, corrCoeff = 0.05, corrCoeff2 = 1;

    public DcMotor fl = null, fr = null, bl = null, br = null;
    public RevIMU imu = null;

    public int cameraMonitorViewId;
    public OpenCvCamera phoneCam = null;
    public BlueVisionRGBNoTele p;

    public CompBotV3() {}

    public void init(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(hardwareMap,"imu");
        imu.init(parameters);
    }
    public void init(HardwareMap h, boolean cameraInit) {
        if(cameraInit) {
            cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            p = new BlueVisionRGBNoTele();
            phoneCam.setPipeline(p);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened() { phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT); }

                //@Override
                public void onError(int errorCode) {}
            });
        }
        init(h);
    }
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void AEncDrive(double dForward, double dStrafe, double sForward, double sStrafe) { // d = distance, s = speed
        // Set the target positions of each motor
        fl.setTargetPosition(fl.getCurrentPosition() + (int) -(distanceK*(dForward+dStrafe))); // distanceK is a conversion factor to convert linear distance to motor clicks;
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (distanceK*(dForward-dStrafe))); // The distance each wheel needs to travel is just the sum of the
        bl.setTargetPosition(bl.getCurrentPosition() + (int) -(distanceK*(dForward-dStrafe))); // distances the wheel would need to travel to do the strafing and
        br.setTargetPosition(br.getCurrentPosition() + (int) (distanceK*(dForward+dStrafe))); // forward/back distances separately
        runToPositionMode(); // Set motors to RUN_TO_POSITION mode - they will automatically spin in the direction of the set position
        double initialHeading = imu.getHeading(), error=0; // Create variables for the gyro correction, and measure the initial angle the robot is facing
        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) {
            error = imu.getHeading() - initialHeading; // Calculate the deviation from the initial angle of the robot using the gyro
            if(fl.isBusy()) { fl.setPower(-(sForward + sStrafe) + corrCoeff*error); // DCMotor.isBusy is a boolean variable signifying whether the motor has finished moving to the position
            } else { fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error, -1, 1));
            }if(fr.isBusy()) { fr.setPower(sForward - sStrafe - corrCoeff*error); // This code looks complicated but it's simple
            } else { fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            }if(bl.isBusy()) { bl.setPower(-(sForward - sStrafe) + corrCoeff*error); // If the motor is not finished, apply the given speed + a correction based on the angle error
            } else { bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            } if(br.isBusy()) { br.setPower(sForward + sStrafe - corrCoeff*error); // If the motor is finished, apply only the correction (flip flops signs because we're in RUN_TO_POSITION mode)
            } else { br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            }
        }
        useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode
    }

    public void driveRobotCentric(double x, double y, double turn) {
        fl.setPower(MathUtils.clamp(-(y + x) + turn,-1,1));
        fr.setPower(MathUtils.clamp(y - x - turn,-1,1));
        bl.setPower(MathUtils.clamp(-(y - x) + turn,-1,1));
        br.setPower(MathUtils.clamp(y + x - turn,-1,1));
    }
    public void gyroTurn(double turn, double sTurn) { // turn is degrees
        useEncoders();
        double expectedHeading = imu.getHeading() + turn, error;
        do {
            error = imu.getHeading() - expectedHeading;
            driveRobotCentric(0,0, (error > 20) ? (Math.signum(error) * sTurn) : ((error / 20) * sTurn));
        } while (Math.abs(error) > 0.01);
    }

    public void runToPositionMode() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void useEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopAndResetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void runMotorTime(DcMotor m, double p, long t) {
        ElapsedTime e = new ElapsedTime();
        while(e.milliseconds() < t) {
            m.setPower(p);
        }
        m.setPower(0);
    }
}
