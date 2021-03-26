package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.background.Cancel;
import org.firstinspires.ftc.teamcode.toolkit.background.Odometry;
import org.firstinspires.ftc.teamcode.toolkit.background.UpliftTelemetry;
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

public class UpliftRobot {

    public LinearOpMode opMode;

    public UpliftTelemetry upliftTelemetry;
    public Cancel cancelClass;
    public Odometry odometry;
    public VelocityData velocityData;
    public boolean driverCancel = false;
    public boolean operatorCancel = false;

    public DriveSubsystem driveSub;
    public TransferSubsystem transferSub;
    public IntakeSubsystem intakeSub;
    public ShooterSubsystem shooterSub;
    public FlickerSubsystem flickerSub;
    public WobbleSubsystem wobbleSub;

    public HardwareMap hardwareMap;
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx shooter1, shooter2;
    public DcMotor intake;
    public DcMotorEx transfer;
    public Servo intakeLifter;
    public Servo wobbleLeft, wobbleRight;
    public Servo flicker, clamp;
    public Servo sweeperJoint, stick;
    public CRServo sweeperLeft;
    public CRServo sweeperRight;
    public DigitalChannel digitalTouchBottom, digitalTouchTop;
    public DistanceSensor shooterSensor;

    public BNO055IMU imu;

    public OpenCvCamera camera;
    WebcamName webcamName;
    public RingDetector ringDetector;

    public double worldX = 0;
    public double worldY = 0;
    public double rawAngle = 0;
    public double worldAngle = 0;
    public int shotCount = 0;
    public double constant = 0;
    public boolean slowMode = false;
    public boolean intakeToggle = false;
    public double shooter1Vel = -1;
    public double shooter2Vel = -1;

    public double highGoalVelocity = 2200;
    public double powerShotVelocity = 1050;
    public double kP = 50;
    public double kI = 0;
    public double kD = 0;
    public double kF = 15;

    public File odometryFileWorldX, odometryFileWorldY, odometryFileWorldAngle, transferFile;

    public boolean driveInitialized, flickerInitialized, intakeInitialized, shooterInitialized, transferInitialized, wobbleInitialized, imuInitialized, visionInitialized;

    // values specific to the drivetrain
    public static double ticksPerQuarterRotation = 720;
    public static double wheelRadius = 19/25.4; // in inches
    public static double wheelCircumference = wheelRadius * (2 * Math.PI); // in inches
    public static double COUNTS_PER_INCH = (720 * 4) / wheelCircumference;
    public static double robotEncoderWheelDistance = 15.7;
    public static double horizontalEncoderInchesPerDegreeOffset = 0.0275;

    // robot constructor
    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
        initSubsystems();
        initBackground();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;

        try {
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

            driveInitialized = true;
        } catch (Exception ex) {
            driveInitialized = false;
            opMode.telemetry.addData("Drivetrain initialization failed: ", ex.getMessage());
        }

        try {
            flicker = hardwareMap.get(Servo.class, "flicker");
            flickerInitialized = true;
        } catch (Exception ex) {
            flickerInitialized = false;
            opMode.telemetry.addData("Flicker initialization failed: ", ex.getMessage());
        }

        try {
            transfer = hardwareMap.get(DcMotorEx.class, "transfer");
            transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            digitalTouchBottom = hardwareMap.get(DigitalChannel.class, "touch_bottom");
            digitalTouchBottom.setMode(DigitalChannel.Mode.INPUT);
//            digitalTouchTop = hardwareMap.get(DigitalChannel.class, "touch_top");
//            digitalTouchTop.setMode(DigitalChannel.Mode.INPUT);

            transferInitialized = true;
        } catch (Exception ex) {
            transferInitialized = false;
            opMode.telemetry.addData("Transfer initialization failed: ", ex.getMessage());
        }

        try {
            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter_2");
            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
            shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

            shooterSensor = hardwareMap.get(DistanceSensor.class, "shooter_sensor");

            shooterInitialized = true;
        } catch (Exception ex) {
            shooterInitialized = false;
            opMode.telemetry.addData("Shooter initialization failed: ", ex.getMessage());
        }

        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeLifter = hardwareMap.get(Servo.class, "intake_lifter");

            sweeperJoint = hardwareMap.get(Servo.class, "sweeper_joint");
            sweeperLeft = hardwareMap.get(CRServo.class, "sweeper_left");
            sweeperRight = hardwareMap.get(CRServo.class, "sweeper_right");
            stick = hardwareMap.get(Servo.class, "stick");

            intakeInitialized = true;
        } catch (Exception ex) {
            intakeInitialized = false;
            opMode.telemetry.addData("Intake initialization failed: ", ex.getMessage());
        }

        try {
            wobbleLeft = hardwareMap.get(Servo.class, "wobble_left");
            wobbleRight = hardwareMap.get(Servo.class, "wobble_right");
            clamp = hardwareMap.get(Servo.class, "clamp");

            wobbleInitialized = true;
        } catch (Exception ex) {
            wobbleInitialized = false;
            opMode.telemetry.addData("Wobble initialization failed: ", ex.getMessage());
//            opMode.telemetry.update();
        }

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(parameters);

//        byte AXIS_MAP_CONFIG_BYTE = 0x6; //swap x and z axis
//        byte AXIS_MAP_SIGN_BYTE = 0x1; //negate z axis
//
//        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0xFF);
//        safeSleep(100);
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0xFF);
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
//        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//        safeSleep(100);

            imuInitialized = true;
        } catch (Exception ex) {
            imuInitialized = false;
            opMode.telemetry.addData("IMU initialization failed: ", ex.getMessage());
//            opMode.telemetry.update();
        }

        try {
            ringDetector = new RingDetector();
            webcamName = hardwareMap.get(WebcamName.class,"webcam");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            camera.openCameraDevice();
            camera.setPipeline(ringDetector);
            camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        } catch (Exception ex) {
            visionInitialized = false;
            opMode.telemetry.addData("Vision initialization failed: ", ex.getMessage());
        }

        // setup file system
        odometryFileWorldX = AppUtil.getInstance().getSettingsFile("odometryX.txt");
        odometryFileWorldY = AppUtil.getInstance().getSettingsFile("odometryY.txt");
        odometryFileWorldAngle = AppUtil.getInstance().getSettingsFile("odometryTheta.txt");
        transferFile = AppUtil.getInstance().getSettingsFile("transferFile.txt");

    }

    public void initSubsystems() {
        if(driveInitialized) {
            driveSub = new DriveSubsystem(this);
        }
        if(transferInitialized) {
            transferSub = new TransferSubsystem(this);
        }
        if(intakeInitialized) {
            intakeSub = new IntakeSubsystem(this);
        }
        if(shooterInitialized) {
            shooterSub = new ShooterSubsystem(this);
            shooterSub.setShooterPIDF(kP, kI, kD, kF);
        }
        if(flickerInitialized) {
            flickerSub = new FlickerSubsystem(this);
        }
        if(wobbleInitialized) {
            wobbleSub = new WobbleSubsystem(this);
        }
    }

    public void initBackground() {
        upliftTelemetry = new UpliftTelemetry(this);
        upliftTelemetry.enable();
        cancelClass = new Cancel(this);
        cancelClass.enable();
        if(driveInitialized) {
            odometry = new Odometry(this);
            odometry.enable();
        }
        if(shooterInitialized) {
            velocityData = new VelocityData(this);
            velocityData.enable();
        }
    }

    public void writePositionToFiles() {
        ReadWriteFile.writeFile(odometryFileWorldX, String.valueOf(worldX));
        ReadWriteFile.writeFile(odometryFileWorldY, String.valueOf(worldY));
        ReadWriteFile.writeFile(odometryFileWorldAngle, String.valueOf(worldAngle));
    }

    public void readPositionFiles() {
        String xStr = ReadWriteFile.readFile(odometryFileWorldX).trim();
        String yStr = ReadWriteFile.readFile(odometryFileWorldY).trim();
        String angleStr = ReadWriteFile.readFile(odometryFileWorldAngle).trim();
        if(!xStr.isEmpty()){
            worldX = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldX).trim());
        }
        if(!yStr.isEmpty()) {
            worldY = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldY).trim());
        }
        if(!angleStr.isEmpty()) {
            worldAngle = Double.parseDouble(ReadWriteFile.readFile(odometryFileWorldAngle).trim());
        }
    }

    public void clearPositionFiles() {
        ReadWriteFile.writeFile(odometryFileWorldX, String.valueOf(0.0));
        ReadWriteFile.writeFile(odometryFileWorldY, String.valueOf(0.0));
        ReadWriteFile.writeFile(odometryFileWorldAngle, String.valueOf(0.0));
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
        if(driveInitialized) {
            odometry.stop();
        }
        if(shooterInitialized) {
            velocityData.stop();
        }
        cancelClass.stop();
    }

    public enum ShootingState {
        IDLE,
        PREPARING_HIGHGOAL,
        PREPARING_POWERSHOT,
        SHOOTING_HIGHGOAL,
        SHOOTING_PS1,
        SHOOTING_PS2,
        SHOOTING_PS3,
        DONE_PS1,
        DONE_PS2,
        DONE_PS3,
        DONE_SHOOTING,
    }
    public ShootingState shootingState = ShootingState.IDLE;
    public void setShootingState(ShootingState state) {
        shootingState = state;
    }

    public enum TransferState {
        DOWN,
        MOVING,
        UP,
        DEFAULT
    }
    public TransferState transferState = TransferState.DEFAULT;
    public void setTransferState(TransferState state) {
        transferState = state;
    }

}
