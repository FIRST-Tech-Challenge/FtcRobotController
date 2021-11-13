package org.firstinspires.ftc.teamcode.Enhancement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enhancement.Config.GamePadConfig;
import org.firstinspires.ftc.teamcode.Enhancement.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Control.Control;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * The big umbrella subsystem.
 *
 * <p>This class starts with variable initializations, then it has initialization functions, has some get and set functions.</p>
 */
public class Robot {
    public final GamePadConfig gamePadConfig = new GamePadConfig();
    private final HardwareMap hardwareMap;
    private final LinearOpMode opMode;
    private final Telemetry oldTelemetry;
    private final QuickTelemetry telemetry;
    private final boolean auto;

    // DC Motors
    public DcMotorEx frontLeftDriveMotor;
    public DcMotorEx frontRightDriveMotor;
    public DcMotorEx rearRightDriveMotor;
    public DcMotorEx rearLeftDriveMotor;
    public DcMotorEx intake;

    // Odometry
    public List<LynxModule> allHubs;
    public DigitalChannel odometryRA;
    public DigitalChannel odometryRB;
    public DigitalChannel odometryBA;
    public DigitalChannel odometryBB;
    public DigitalChannel odometryLA;
    public DigitalChannel odometryLB;
    public int odRCount = 0;
    public int odBCount = 0;
    public int odLCount = 0;


    //Sensors
    public BNO055IMU imu;
    // Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;
    private ElapsedTime timer;
    private double joystickDeadZone = 0.1;

    /**
     * Note that this method only changes some variables the real work is done in {@link #init()}
     *
     * @param opMode the operational mode, the telemetry and hardware map is gotten from this
     * @param timer  an timer
     * @see LinearOpMode
     * @see ElapsedTime
     */
    public Robot(LinearOpMode opMode, ElapsedTime timer, boolean auto) {
        this.opMode = opMode;
        this.oldTelemetry = opMode.telemetry;
        this.telemetry = new QuickTelemetry(oldTelemetry);
        this.hardwareMap = opMode.hardwareMap;
        this.timer = timer;
        this.auto = auto;

        this.telemetry.telemetry(1, MainConfig.getName(), "v" + MainConfig.getVersion());

        this.telemetry.telemetry(4, "Config", "Main Config Variables:");
        this.telemetry.telemetry(4, "name", "name = " + MainConfig.getName());
        this.telemetry.telemetry(4, "version", "version = " + MainConfig.getVersion());
        this.telemetry.telemetry(4, "allianceColor", "allianceColor = " + MainConfig.getAllianceColor());
        this.telemetry.telemetry(4, "debug", "debug = " + MainConfig.getDebug());
        this.telemetry.telemetry(4, "debugTarget", "debugTarget = " + MainConfig.getDebugTarget());
        this.telemetry.telemetry(4, "logLevel", "logLevel = " + MainConfig.getLogLevel());
        this.telemetry.telemetry(4, "initMinorSubsystems", "initMinorSubsystems = " + MainConfig.getInitSubsystems());
        this.telemetry.telemetry(4, "initMechanical", "initMechanical = " + MainConfig.getInitMechanical());
        this.telemetry.telemetry(4, "initGetGamePadInputs", "initGetGamePadInputs = " + MainConfig.getInitGamePadInputs());
        this.oldTelemetry.addLine();
        this.telemetry.telemetry(1, "Finished Basic init", "Finished Basic Initialization");

        this.telemetry.telemetry(2, "running init()", "Running init()");

        init();

        this.telemetry.telemetry(1, "running init()", "Finished running init()");
    }

    /**
     * Initializes everything
     *
     * <p>It first initializes some mechanical things, then it initializes the subsystems.</p>
     *
     * @see #initMechanical()
     */
    public void init() {
        if (MainConfig.getInitGamePadInputs()) {
            this.telemetry.telemetry(3, "Get gamePad inputs", "gamePad init started");
            getGamePadInputs();
            this.telemetry.telemetry(2, "Get gamePad inputs", "gamePad init finished");
        } else {
            telemetry.telemetry(1, "Get gamePad inputs", " gamePad init skipped");
        }

        if (MainConfig.getInitMechanical()) {
            telemetry.telemetry(3, "Hardware init", " Hardware init started");
            initMechanical(); // mechanical stuff
            telemetry.telemetry(2, "Hardware init", " Hardware init finished");
        } else {
            telemetry.telemetry(1, "Hardware init", " Hardware init skipped");
        }

        if (MainConfig.getInitSubsystems()) {
            telemetry.telemetry(2, "Subsystems init", " Subsystems init started");
            initSubsystems();
            telemetry.telemetry(1, "Subsystems init", " Subsystems init finished");
        } else {
            telemetry.telemetry(1, "Subsystems init", " Subsystems init skipped");
        }

    }

    private void initMechanical() {
        // DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.telemetry(4, "Motors", "Stopping all motors");

        // Stops all the motors
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        HardwareMap.DeviceMapping<Servo> servo = hardwareMap.servo;

        allHubs = hardwareMap.getAll(LynxModule.class);

//        odometryRA  = hardwareMap.get(DigitalChannel.class, "odra");
//        odometryRB = hardwareMap.get(DigitalChannel.class, "odrb");
//        odometryBA  = hardwareMap.get(DigitalChannel.class, "odba");
//        odometryBB = hardwareMap.get(DigitalChannel.class, "odbb");
//        odometryLA  = hardwareMap.get(DigitalChannel.class, "odla");
//        odometryLB = hardwareMap.get(DigitalChannel.class, "odlb");
//
//        odometryRA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryRB.setMode(DigitalChannel.Mode.INPUT);
//        odometryBA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryBB.setMode(DigitalChannel.Mode.INPUT);
//        odometryLA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryLB.setMode(DigitalChannel.Mode.INPUT);

        initIMU();

    }

    private void initSubsystems() {
        if (MainConfig.getInitSubsystemDrive()) {
            // Drive
            telemetry.telemetry(3, "Mode", " Drive init started");
            List<DcMotorEx> motors = new ArrayList<>(4);
            motors.add(frontLeftDriveMotor);
            motors.add(frontRightDriveMotor);
            motors.add(rearLeftDriveMotor);
            motors.add(rearRightDriveMotor);

            drive = new Drive(this.getQuickTelemetry("Drive"), hardwareMap, timer, motors, imu);
//        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.telemetry(2, "Mode", " Drive init finished");
        }

        if (MainConfig.getInitSubsystemVision() || auto) {
            telemetry.telemetry(3, "Mode", " Vision init started");
            vision = new Vision(this.getQuickTelemetry("Vision"), hardwareMap, timer);
            telemetry.telemetry(2, "Mode", " Vision init finished");
        }

        if (MainConfig.getInitSubsystemControl()) {
            telemetry.telemetry(3, "Mode", " Control init started");
            control = new Control(this.getQuickTelemetry("Control"), hardwareMap, timer, intake);
            telemetry.telemetry(2, "Mode", " Control init finished");
        }
    }

    /**
     * <p>Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port on a Core
     * Device Interface Module, configured to be a sensor of type "AdaFruit IMU", and named "imu".</p>
     */
    private void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.telemetry(3, "Mode", " IMU initializing...");
        imu.initialize(parameters);
        telemetry.telemetry(4, "Mode", " IMU calibrating...");
        // make sure the imu gyro is calibrated before continuing.
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }
        telemetry.telemetry(2, "Mode", " IMU calibration finished...");
    }

    public LinearOpMode getOpMode() {
        return this.opMode;
    }

    /**
     * Gets the telemetry.
     * <p>
     * Try to use {@link #getQuickTelemetry()} instead
     *
     * @return The telemetry
     * @see Telemetry
     * @see #getQuickTelemetry()
     */
    public Telemetry getTelemetry() {
        return this.oldTelemetry;
    }

    /**
     * @return Quick Telemetry
     * @see QuickTelemetry
     */
    public QuickTelemetry getQuickTelemetry() {
        return this.telemetry;
    }

    public QuickTelemetry getQuickTelemetry(String file) {
        return this.telemetry.newQuickTelemetryFile(file);
    }

    public ElapsedTime getTimer() {
        return this.timer;
    }

    public void getGamePadInputs() {
        gamePadConfig.mapGamePadInputs(this);
    }

    public double joystickDeadZoneCorrection(double joystickInput) {
        double joystickOutput;
        if (joystickInput > joystickDeadZone) {
            joystickOutput = (joystickInput - joystickDeadZone) / (1.0 - joystickDeadZone);
        } else if (joystickInput > -joystickDeadZone) {
            joystickOutput = 0.0;
        } else {
            joystickOutput = (joystickInput + joystickDeadZone) / (1.0 - joystickDeadZone);
        }
        return joystickOutput;
    }


    public void setFrontLeftDriveMotor(double power) {
        frontLeftDriveMotor.setPower(power);
    }

    public void setFrontRightDriveMotor(double power) {
        frontRightDriveMotor.setPower(power);
    }

    public void setRearLeftDrivePower(double power) {
        rearLeftDriveMotor.setPower(power);
    }

    public void setRearRightDriveMotor(double power) {
        rearRightDriveMotor.setPower(power);
    }
}
