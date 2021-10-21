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
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Config.GamePadConfig;
import org.firstinspires.ftc.teamcode.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Subsystems.Control.Control;

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
    private final String name = "Freight Mover"; // TODO: Better name needed
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
    public DcMotorEx launch1;
    public DcMotorEx launch2a;
    public DcMotorEx launch2b;
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
     * @throws IOException Might throw it.
     *
     * @see LinearOpMode
     * @see ElapsedTime
     * @see AllianceColor
     */
    public Robot(LinearOpMode opMode, ElapsedTime timer) throws IOException {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.oldTelemetry = opMode.telemetry;
        this.telemetry = new QuickTelemetry(oldTelemetry);
        this.timer = timer;

        init();
    }

    /**
     * Initializes everything
     *
     * <p>It first initializes some mechanical things, then it initializes the subsystems.</p>
     * @throws IOException
     *
     * @see #initMechanical()
     */
    public void init() throws IOException {
        initMechanical(); // mechanical stuff
        // Drive
        telemetry.telemetry("Mode", " drive/control initializing...");
        List<DcMotorEx> dcMotorExList = new ArrayList<>(4);
        dcMotorExList.add(frontLeftDriveMotor);
        dcMotorExList.add(frontRightDriveMotor);
        dcMotorExList.add(rearLeftDriveMotor);
        dcMotorExList.add(rearRightDriveMotor);

        drive = new Drive(this, dcMotorExList, intake, launch1, launch2b, imu);
//        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.telemetry("Mode", " vision initializing...");
        try {
            vision = new Vision(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        telemetry.telemetry("Mode", " control initializing...");
        control = new Control(this);

    }

    public void initMechanical() {
        // DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");

        // frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // rearRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launch1 = (DcMotorEx) hardwareMap.dcMotor.get("launch1");
        launch1.setDirection(DcMotorSimple.Direction.REVERSE);
        launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch1.setPower(0.0);

        launch2a = (DcMotorEx) hardwareMap.dcMotor.get("launch2a");
        launch2a.setDirection(DcMotorSimple.Direction.REVERSE);
        launch2a.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2a.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch2a.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch2a.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2a.setPower(0.0);

        launch2b = (DcMotorEx) hardwareMap.dcMotor.get("launch2b");
        launch2b.setDirection(DcMotorSimple.Direction.REVERSE);
        launch2b.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2b.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch2b.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch2b.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2b.setPower(0.0);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);

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

        telemetry.telemetry("Mode", " IMU initializing...");
        imu.initialize(parameters);
        telemetry.telemetry("Mode", " IMU calibrating...");
        // make sure the imu gyro is calibrated before continuing.
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated())
        {
            opMode.sleep(50);
            opMode.idle();
        }
    }

    public String getName() {
        return this.name;
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


}
