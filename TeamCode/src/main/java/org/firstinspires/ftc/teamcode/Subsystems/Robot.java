package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.GamePadConfig;
import org.firstinspires.ftc.teamcode.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Control.Control;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * The big umbrella subsystem.
 *
 * <p>This class starts with variable initializations</p></p>
 */
public class Robot extends Subsystem {
    private final AllianceColor allianceColor = MainConfig.getAllianceColor();
    private final String name = MainConfig.getName();
    private final String version = MainConfig.getVersion();
    private final boolean debug = MainConfig.getDebug();
    private final int logLevel = MainConfig.getLogLevel();
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    private Telemetry oldTelemetry;
    private QuickTelemetry telemetry;
    private ElapsedTime timer;

    // DC Motors
    public DcMotorEx frontLeftDriveMotor;
    public DcMotorEx frontRightDriveMotor;
    public DcMotorEx rearRightDriveMotor;
    public DcMotorEx rearLeftDriveMotor;

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


    /**
     * Control Hub
     *
     * fr        0
     * br        1
     * launch2   2
     * launch1   3
     *
     * --------------------
     * Expansion Hub 2
     *
     * fl        0
     * bl        1
     * intake    2
     *
     * feeding          0
     * left elevator    1
     * wbc1             2
     * wbc2             3
     * right elevator   4
     */

    //Sensors
    public BNO055IMU imu;

    GamePadConfig gamePadConfig = new GamePadConfig();

    private double joystickDeadZone = 0.1;

    // Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;


    /**
     * Note that this method only changes some variables the real work is done in {@link #init()}
     *
     * @param opMode the operational mode, the telemetry and hardware map is gotten from this
     * @param timer an timer
     *
     * @see LinearOpMode
     * @see ElapsedTime
     * @see AllianceColor
     */
    public Robot(LinearOpMode opMode, ElapsedTime timer) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.oldTelemetry = opMode.telemetry;
        this.telemetry = new QuickTelemetry(oldTelemetry);
        this.timer = timer;


        this.telemetry.telemetry(1, "running init()", "Running init()");

        getGamePadInputs();
        // init();
    }

    /**
     * Initializes everything
     *
     * <p>It first initializes some mechanical things, then it initializes the subsystems.</p>
     *
     * @see #initMechanical()
     */
    public void init() {
        telemetry.telemetry(2, "Mode", " Hardware init started");
        initMechanical(); // mechanical stuff
        telemetry.telemetry(2, "Mode", " Hardware init finished");

        // Drive
        telemetry.telemetry(2, "Mode", " Drive init started");
        List<DcMotorEx> motors = new ArrayList<>(4);
        motors.add(frontLeftDriveMotor);
        motors.add(frontRightDriveMotor);
        motors.add(rearLeftDriveMotor);
        motors.add(rearRightDriveMotor);

        drive = new Drive(this, motors, imu);
//        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.telemetry(1, "Mode", " Drive init finished");

        telemetry.telemetry(2, "Mode", " Vision init started");
        try {
            vision = new Vision(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.telemetry(1, "Mode", " Vision init finished");

        telemetry.telemetry(2, "Mode", " Control init started");
        control = new Control(this);
        telemetry.telemetry(2, "Mode", " Control init finished");
    }

    public void initMechanical() {
        // DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Servos
//        clawDeploy = hardwareMap.servo.get("clawDeploy");
//        claw = hardwareMap.servo.get("claw");
//        elevator1 = hardwareMap.crservo.get("e1");
//        elevator2 = hardwareMap.crservo.get("e2");
//
//        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
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


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.telemetry(2, "Mode", " IMU initializing...");
        imu.initialize(parameters);
        telemetry.telemetry(2, "Mode", " IMU calibrating...");
        // make sure the imu gyro is calibrated before continuing.
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }
    }

    public String getName() {
        return this.name;
    }

    public int getLogLevel() {
        return this.logLevel;
    }

    public LinearOpMode getOpMode() {
        return this.opMode;
    }

    /**
     * Gets the telemetry.
     *
     * Try to use {@link #getQuickTelemetry()} instead
     * @return The telemetry
     *
     * @see #getQuickTelemetry()
     */
    public Telemetry getTelemetry() {
        return this.oldTelemetry;
    }

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

    public double joystickDeadzoneCorrection(double joystickInput) {
        double joystickOutput;
        if (joystickInput > joystickDeadZone) {
            joystickOutput = (joystickInput - joystickDeadZone) / (1.0 - joystickDeadZone);
        }
        else if (joystickInput > -joystickDeadZone) {
            joystickOutput = 0.0;
        }
        else {
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

    public void setBackLeftPower(double power) {
        rearLeftDriveMotor.setPower(power);
    }

    public void setRearLeftDriveMotor(double power) {
        rearRightDriveMotor.setPower(power);
    }
}
