package org.firstinspires.ftc.teamcode.CompBot;

// Hardware class for Viridian's competition bot
// Version 1.0.0

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.vision.BlueVisionRGBNoTele;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CompBotHW {
    public Motor fl = null, fr = null, bl = null, br = null, intake = null, spin = null, lift = null;
    public MecanumDrive m;

    public Servo bucket;

    public RevIMU imu = null;

    public int cameraMonitorViewId;
    public OpenCvCamera phoneCam = null;

    public CompBotHW() {}

    public void init(HardwareMap h) {
        fl = new Motor(h,"fl");
        fr = new Motor(h,"fr");
        bl = new Motor(h,"bl");
        br = new Motor(h,"br");
        m = new MecanumDrive(fl,fr,bl,br);

        m.stop();

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        intake = new Motor(h, "intake");
        intake.set(0);
        intake.setInverted(false);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        spin = new Motor(h,"spin");
        spin.set(0);
        spin.setInverted(false);
        spin.setRunMode(Motor.RunMode.VelocityControl);
        spin.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lift = new Motor(h,"lift");
        lift.set(0);
        lift.setInverted(false);
        lift.setRunMode(Motor.RunMode.VelocityControl);
        lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(h,"imu");
        imu.init(parameters);

        bucket = h.get(Servo.class, "bucket");
        bucket.setPosition(0);

    }
    public void init(HardwareMap h, boolean cameraInit) {
        if(cameraInit) {
            cameraMonitorViewId = h.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", h.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            BlueVisionRGBNoTele p = new BlueVisionRGBNoTele();
            phoneCam.setPipeline(p);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened() { phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT); }

                @Override
                public void onError(int errorCode) {}
            });
        }
        init(h);
    }
}
