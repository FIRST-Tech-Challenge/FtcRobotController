package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
import org.firstinspires.ftc.teamcode.toolkit.background.VelocityData;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

public class UpliftRobot {

    public LinearOpMode opMode;

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
    public DcMotor intake, transfer;
    public Servo wobbleTop, wobbleBottom;
    public Servo flicker, clamp;
    public DigitalChannel digitalTouchBottom, digitalTouchTop;
//    public DistanceSensor intakeSensor;

    public BNO055IMU imu;

    public OpenCvCamera camera;
    WebcamName webcamName;
    public RingDetector ringDetector;

    public double worldX = 0;
    public double worldY = 0;
    public double rawAngle = 0;
    public double worldAngle = 0;
    public int count = 0;
    public double constant = 0;
    public boolean slowMode = false;
    public double shooter1SmoothVel = -1;
    public double shooter2SmoothVel = -1;
    public double shooter1RawVel = -1;
    public double shooter2RawVel = -1;
    public double highGoalVelocity = 1900;
    public double powerShotVelocity = 825;

    public File odometryFileWorldX, odometryFileWorldY, odometryFileWorldAngle, transferFile;

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
        initGameState();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;

        //initialize the motors into the hardware map
        leftFront = hardwareMap.get(DcMotor.class,"lf_motor");//Declares two left motors
        leftBack = hardwareMap.get(DcMotor.class,"lb_motor");
        rightFront = hardwareMap.get(DcMotor.class,"rf_motor"); //Declares two right motors
        rightBack = hardwareMap.get(DcMotor.class,"rb_motor");

        flicker = hardwareMap.get(Servo.class,"flicker");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter_2");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        intakeSensor = hardwareMap.get(DistanceSensor.class,"d1");
        wobbleTop = hardwareMap.get(Servo.class, "wobble1");
        wobbleBottom = hardwareMap.get(Servo.class, "wobble2");
        clamp = hardwareMap.get(Servo.class, "clamp");
        digitalTouchBottom = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouchBottom.setMode(DigitalChannel.Mode.INPUT);
        digitalTouchTop = hardwareMap.get(DigitalChannel.class, "touch_top");
        digitalTouchTop.setMode(DigitalChannel.Mode.INPUT);

        //setup imu (gyro)
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

        ringDetector = new RingDetector();
        webcamName = hardwareMap.get(WebcamName.class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(ringDetector);
        camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        // setup file system
        odometryFileWorldX = AppUtil.getInstance().getSettingsFile("odometryX.txt");
        odometryFileWorldY = AppUtil.getInstance().getSettingsFile("odometryY.txt");
        odometryFileWorldAngle = AppUtil.getInstance().getSettingsFile("odometryTheta.txt");
        transferFile = AppUtil.getInstance().getSettingsFile("transferFile.txt");

        //setup the motors
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

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void initSubsystems() {
        driveSub = new DriveSubsystem(this);
        transferSub = new TransferSubsystem(this);
        intakeSub = new IntakeSubsystem(this);
        shooterSub = new ShooterSubsystem(this);
        flickerSub = new FlickerSubsystem(this);
        wobbleSub = new WobbleSubsystem(this);
    }

    public void initBackground() {
        cancelClass = new Cancel(this);
        odometry = new Odometry(this);
        velocityData = new VelocityData(this);
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
        odometry.stopUpdatingPos();
        velocityData.stopUpdatingVelocity();
        cancelClass.stopUpdatingCancel();
    }

    public void initGameState() {
        if(opMode instanceof UpliftAuto) {
            setGameState(GameState.AUTO);
        } else {
            setGameState(GameState.TELEOP);
        }
    }

    public ShootingState shootingState;
    public enum ShootingState {
        IDLE,
        PREPARING_TO_SHOOT,
        SHOOTING,
        DONE_SHOOTING,
    }
    public void setShootingState(ShootingState state) {
        shootingState = state;
    }

    public GameState gameState;
    public enum GameState {
        AUTO,
        TELEOP,
        ENDGAME
    }
    public void setGameState(GameState state) {
        gameState = state;
    }

}
