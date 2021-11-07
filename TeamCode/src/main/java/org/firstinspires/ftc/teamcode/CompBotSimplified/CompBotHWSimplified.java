package org.firstinspires.ftc.teamcode.CompBotSimplified;

// Hardware class for Viridian's competition bot, but simplified
// Version 1.0.0

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.BlueVisionYCbCrImproved;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CompBotHWSimplified {
    public final static double distanceK = 4554.756025274308733, corrCoeff = 0.03;

    public Motor fl = null, fr = null, bl = null, br = null;
    public MecanumDrive m;

    public RevIMU imu = null;

    public int cameraMonitorViewId;
    public OpenCvCamera phoneCam = null;
    public BlueVisionYCbCrImproved p;

    public CompBotHWSimplified() {}

    public void init(HardwareMap h) {
        fl = new Motor(h,"fl");
        fr = new Motor(h,"fr");
        bl = new Motor(h,"bl");
        br = new Motor(h,"br");
        m = new MecanumDrive(fl,fr,bl,br);

        m.stop();

        rawPower();

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fl.setDistancePerPulse(distanceK);
        fr.setDistancePerPulse(distanceK);
        bl.setDistancePerPulse(distanceK);
        br.setDistancePerPulse(distanceK);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(h,"imu");
        imu.init(parameters);


    }
    public void init(HardwareMap h, boolean cameraInit) {
        if(cameraInit) {
            cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            p = new BlueVisionYCbCrImproved();
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

    // distances are in inches
    public void encoderDrive(double dStrafe, double dForward, double sStrafe, double sForward) {
        positionControl();
        setTargetDist(new double[]{dForward+dStrafe, dForward-dStrafe, dForward-dStrafe, dForward+dStrafe});
        while(!(fl.atTargetPosition() || fr.atTargetPosition() || bl.atTargetPosition() || br.atTargetPosition())) {
            m.driveRobotCentric(sStrafe,sForward,0);
        }
        m.stop();
        velocityControl();
    }
    public void assistedEncoderDrive(double dStrafe, double dForward, double sStrafe, double sForward) {
        positionControl();
        setTargetDist(new double[]{dForward+dStrafe, dForward-dStrafe, dForward-dStrafe, dForward+dStrafe});
        double initialHeading = imu.getHeading(), error;
        while(!(fl.atTargetPosition() || fr.atTargetPosition() || bl.atTargetPosition() || br.atTargetPosition())) {
            error = imu.getHeading() - initialHeading;
            m.driveRobotCentric(sStrafe,sForward,corrCoeff*error);
        }
        m.stop();
        rawPower();
    }
    public void gyroTurn(double turn, double sTurn) { // turn is degrees
        velocityControl();
        double expectedHeading = imu.getHeading() + turn, error;
        do {
            error = imu.getHeading() - expectedHeading;
            m.driveRobotCentric(0,0, (error > 20) ? (Math.signum(error) * sTurn) : ((error / 20) * sTurn));
        } while (Math.abs(error) > 0.01);
        m.stop();
        rawPower();
    }

    public void positionControl() {
        fl.setRunMode(Motor.RunMode.PositionControl);
        fr.setRunMode(Motor.RunMode.PositionControl);
        bl.setRunMode(Motor.RunMode.PositionControl);
        br.setRunMode(Motor.RunMode.PositionControl);
    }

    public void velocityControl() {
        fl.setRunMode(Motor.RunMode.VelocityControl);
        fr.setRunMode(Motor.RunMode.VelocityControl);
        bl.setRunMode(Motor.RunMode.VelocityControl);
        br.setRunMode(Motor.RunMode.VelocityControl);
    }

    public void rawPower() {
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
    }

    public void setTargetDist(double[] dist) {
        fl.setTargetDistance(dist[0]);
        fr.setTargetDistance(dist[1]);
        bl.setTargetDistance(dist[2]);
        br.setTargetDistance(dist[3]);
    }
}
