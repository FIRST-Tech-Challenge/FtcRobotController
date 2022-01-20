package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.OdometryPodServos;

/**
 * A Abstract class that provides extra reusable functionality to our opModes <P>
 * The provided functionality:
 * <ul>
 * <li>Easy initialization of every generically initialized subsystem on the robot through associated method calls.</li>
 * <li>Every names of each item in the Hardware map is stored allowing a one line change for a hardware remap</li>
 * <li>It adds the abstract method {@link #opModeMain()} as a entry point for all child classes. Meanwhile, {@link #runOpMode()} sets some stuff up before calling {@link #opModeMain()}</li>
 * </ul>
 */
public abstract class GenericOpModeTemplate extends LinearOpMode {
    /**
     * Name of the front right motor
     */
    public static final String frontRightName = "front_right/vr";
    /**
     * Name of the front left motor
     */
    public static final String frontLeftName = "front_left/vl";
    /**
     * Name of the front left motor
     */
    public static final String backRightName = "back_right/h";
    /**
     * Name of the back left motor
     */
    public static final String backLeftName = "back_left";

    /**
     * Name of the Vertical Left servo
     */
    public static final String verticalLeftServoName = "vertical_left_odometry_servo";
    /**
     * Name of the Vertical Right servo
     */
    public static final String verticalRightServoName = "vertical_right_odometry_servo";
    /**
     * Name of the Horizontal servo
     */
    public static final String horizontalServoName = "horizontal_odometry_servo";

    /**
     * Name of the carousel Spinner
     */
    public static final String carouselSpinnerName = "cs";

    /**
     * Name of the bucket servo
     */
    public static final String bucketServoName = "bucket";
    /**
     * Name of the intake motor
     */
    public static final String intakeMotorName = "intake";
    /**
     * Name of the color sensor
     */
    public static final String bucketColorSensorName = "color_sensor";

    /**
     * Name of the LED string
     */
    public static final String LEDName = "LED";

    /**
     * Name of the Linear Slide drive motor
     */
    public static final String linearSlideMotorName = "linear_slide";

    /**
     * Name of the IMU
     */
    public static final String IMUName = "imu";

    /**
     * The entry point for all child classes of {@link GenericOpModeTemplate}
     *
     * @throws InterruptedException Throws if OpMode is stopped during Execution
     */
    public abstract void opModeMain() throws InterruptedException;

    /**
     * This method calls {@link #opModeMain()}, and catches and rethrows every {@link RuntimeException} with the stack trace as the message to make debugging easier
     *
     * @throws InterruptedException Throws if OpMode is stopped during Execution
     */
    @Override
    public final void runOpMode() throws InterruptedException {
        try {
            opModeMain();
        } catch (RuntimeException e) {
            RobotLog.setGlobalErrorMsg(MiscUtills.getStackTraceAsString(e)); //Appends more information to the error message
        }
    }

    /**
     * Allows the subsystems to look at voltage
     */
    protected RobotVoltageSensor voltageSensor;

    /**
     * Provides Carousel Spinner functionality
     */
    protected CarouselSpinner spinner;

    /**
     * Provides methods for lifting and lowering Odometry servos
     */
    protected OdometryPodServos podServos;

    /**
     * Provides methods for using the intake
     */
    protected ContinuousIntake intake;

    /**
     * Provides methods for using the robot LED's
     */
    protected RevBlinkinLedDriver leds;

    /**
     * Provides methods for using the linear slide
     */
    protected LinearSlide slide;

    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if opMode is stopped during execution
     */
    protected void initAll() throws InterruptedException {
        initOdometryServos();
        initIntake();
        initLEDS();
        checkStop();
        initLinearSlide();
        initVoltageSensor();
        initSpinner();
        if (voltageSensor.getVoltage() < 12.5) {
            RobotLog.addGlobalWarningMessage("Voltage reported by internal sensor less than 12.5V");
        }
    }

    /**
     * Initializes the Odometry Servos
     */
    protected void initOdometryServos() {
        podServos = new OdometryPodServos(hardwareMap, verticalRightServoName, verticalLeftServoName, horizontalServoName);
    }

    /**
     * Initializes the Spinner
     */
    protected void initSpinner() {
        spinner = new CarouselSpinner(hardwareMap, carouselSpinnerName);
    }

    /**
     * Initializes the Intake
     */
    protected void initIntake() {
        intake = new ContinuousIntake(hardwareMap, intakeMotorName, bucketServoName, bucketColorSensorName, true);
        intake.setServoClosed();
    }

    /**
     * Initializes the LED's
     */
    protected void initLEDS() {
        leds = hardwareMap.get(RevBlinkinLedDriver.class, LEDName);
    }

    /**
     * Initializes the voltage sensor
     */
    protected void initVoltageSensor() {
        voltageSensor = new RobotVoltageSensor(hardwareMap);

    }

    /**
     * Initializes the voltage sensor
     *
     * @throws InterruptedException Throws if opMode is stopped during execution
     */
    protected void initLinearSlide() throws InterruptedException {
        {
            DcMotor tmp = hardwareMap.dcMotor.get(linearSlideMotorName);
            tmp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tmp.setPower(.1);
            Thread.sleep(1100);
            tmp.setPower(0);
            tmp.close();
        }
        if (voltageSensor == null) {
            this.initVoltageSensor();
        }
        slide = new LinearSlide(hardwareMap, linearSlideMotorName, voltageSensor, this::opModeIsActive, this::isStopRequested);
    }

    /**
     * Checks to see if the OpMode has been stopped
     *
     * @throws InterruptedException Throws if the OpMode is asked to stop
     */
    protected void checkStop() throws InterruptedException {
        if (this.isStopRequested() || Thread.currentThread().isInterrupted()) {
            throw new InterruptedException();
        }
    }
}
