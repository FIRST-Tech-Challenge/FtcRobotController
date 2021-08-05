package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.background.ShotCounter;
import org.firstinspires.ftc.teamcode.toolkit.background.UpliftTelemetry;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

public class UpliftRobot {

    public LinearOpMode opMode;

    public UpliftTelemetry upliftTelemetry;
    public Odometry odometry;
    public VelocityData velocityData;
    public ShotCounter shotCounter;
    public boolean driverCancel = false, operatorCancel = false;

    public HardwareMap hardwareMap;
    public DcMotor leftFront, leftBack, rightFront, rightBack, intake;
    public DcMotorEx shooter1, shooter2, transfer;
    public Servo intakeLifter, wobbleLeft, wobbleRight, flicker, clamp, sweeperJoint, stick;
    public CRServo sweeperLeft, sweeperRight;
    public DigitalChannel digitalTouchBottom, digitalTouchTop;
    public DistanceSensor shooterSensor;
    public AnalogInput potentiometer;

    public BNO055IMU imu;

    public OpenCvCamera camera;
    WebcamName webcamName;
    public RingDetector ringDetector;

    public double worldX = 0, worldY = 0, rawAngle = 0, worldAngle = 0;
    public double imuAngle = 0;
    public int shotCount = 0;
    public boolean slowMode = false;
    public double shooter1Vel = -1, shooter2Vel = -1;

    public double highGoalVelocity = 1850, powerShotVelocity = 1500, bounceBackVelocity = 1500, autoHighGoalVelocity = 1800;
    public double kP = 50, kI = 0, kD = 0, kF = 15;

    public File odometryFileWorldX, odometryFileWorldY, odometryFileWorldAngle, transferFile, imuFile;

    // values specific to the drivetrain
    public static double wheelRadius = 0.7480315; // in inches
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // in inches
    public static double COUNTS_PER_INCH = (720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 15.7;
    public static double horizontalEncoderInchesPerDegreeOffset = 0.0275;

    // robot constructor
    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
        initBackground();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;

        leftFront = hardwareMap.get(DcMotor.class, "lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class, "lb_motor");
        rightFront = hardwareMap.get(DcMotor.class, "rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class, "rb_motor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flicker = hardwareMap.get(Servo.class, "flicker");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        digitalTouchBottom = hardwareMap.get(DigitalChannel.class, "touch_bottom");
        digitalTouchBottom.setMode(DigitalChannel.Mode.INPUT);
        digitalTouchTop = hardwareMap.get(DigitalChannel.class, "touch_top");
        digitalTouchTop.setMode(DigitalChannel.Mode.INPUT);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter_2");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterSensor = hardwareMap.get(DistanceSensor.class, "shooter_sensor");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLifter = hardwareMap.get(Servo.class, "intake_lifter");

        sweeperJoint = hardwareMap.get(Servo.class, "sweeper_joint");
        sweeperLeft = hardwareMap.get(CRServo.class, "sweeper_left");
        sweeperRight = hardwareMap.get(CRServo.class, "sweeper_right");
        stick = hardwareMap.get(Servo.class, "stick");

        wobbleLeft = hardwareMap.get(Servo.class, "wobble_left");
        wobbleRight = hardwareMap.get(Servo.class, "wobble_right");
        clamp = hardwareMap.get(Servo.class, "clamp");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);


        // if the current opmode is an Auto, setup vision
        if(opMode instanceof UpliftAuto) {
                ringDetector = new RingDetector();
                webcamName = hardwareMap.get(WebcamName.class, "webcam");
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
                camera.openCameraDevice();
                camera.setPipeline(ringDetector);
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        // setup file system
        odometryFileWorldX = AppUtil.getInstance().getSettingsFile("odometryX.txt");
        odometryFileWorldY = AppUtil.getInstance().getSettingsFile("odometryY.txt");
        odometryFileWorldAngle = AppUtil.getInstance().getSettingsFile("odometryTheta.txt");
        transferFile = AppUtil.getInstance().getSettingsFile("transferFile.txt");
        imuFile = AppUtil.getInstance().getSettingsFile("imuFile.txt");

    }

    public void initBackground() {
        upliftTelemetry = new UpliftTelemetry(this);
        upliftTelemetry.enable();
        odometry = new Odometry(this);
        odometry.enable();
        velocityData = new VelocityData(this);
        velocityData.enable();
        shotCounter = new ShotCounter(this);
        shotCounter.enable();
    }

    public void writePositionToFiles() {
        ReadWriteFile.writeFile(odometryFileWorldX, String.valueOf(worldX));
        ReadWriteFile.writeFile(odometryFileWorldY, String.valueOf(worldY));
        ReadWriteFile.writeFile(odometryFileWorldAngle, String.valueOf(worldAngle));
        ReadWriteFile.writeFile(imuFile, String.valueOf(imuAngle));
    }

    public void readPositionFiles() {
        String xStr = ReadWriteFile.readFile(odometryFileWorldX).trim();
        String yStr = ReadWriteFile.readFile(odometryFileWorldY).trim();
        String angleStr = ReadWriteFile.readFile(odometryFileWorldAngle).trim();
        String imuStr = ReadWriteFile.readFile(imuFile).trim();
        if(!xStr.isEmpty()){
            worldX = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldX).trim());
        }
        if(!yStr.isEmpty()) {
            worldY = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldY).trim());
        }
        if(!angleStr.isEmpty()) {
            worldAngle = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldAngle).trim());
        }
        if(!imuStr.isEmpty()) {
            imuAngle = Double.parseDouble(ReadWriteFile.readFile(imuFile).trim());
        }
    }

    public void clearPositionFiles() {
        ReadWriteFile.writeFile(odometryFileWorldX, String.valueOf(0.0));
        ReadWriteFile.writeFile(odometryFileWorldY, String.valueOf(0.0));
        ReadWriteFile.writeFile(odometryFileWorldAngle, String.valueOf(0.0));
        ReadWriteFile.writeFile(imuFile, String.valueOf(0.0));
    }

    public boolean safeSleep(long millis) {
        long initialTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - initialTime < millis) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (driverCancel || operatorCancel || !opMode.opModeIsActive() || opMode.isStopRequested()) {
                return false;
            }
        }
        return true;
    }

    public void stopThreads() {
        odometry.stop();
        velocityData.stop();
    }

}
