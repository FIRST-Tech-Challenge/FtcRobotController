package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.SpaceBar;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.carouselspinner.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake.StateOuttake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.podservos.OdometryPodServos;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.tapemeasureturret.TapeMeasureTurret;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;

import java.util.concurrent.CancellationException;

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
    public static final String frontRightName = "front_right";
    /**
     * Name of the front left motor
     */
    public static final String frontLeftName = "front_left";
    /**
     * Name of the front left motor
     */
    public static final String backRightName = "back_right";
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
    public static final String leftCarouselSpinnerName = "carousel_spinner_left";

    public static final String rightCarouselSpinnerName = "carousel_spinner_right";

    /**
     * Name of the bucket servo
     */
    public static final String bucketServoName = "bucket";
    /**
     * Name of the front intake motor
     */
    public static final String frontIntakeMotorName = "front_intake/vr";

    public static final String backIntakeMotorName = "back_intake/vl";

    public final static String horizontalOdometryName = "h";

    /**
     * Name of the color sensor
     */
    public static final String bucketColorSensorName = "color_sensor";

    /**
     * Name of the left touch sensor in intake
     */
    public static final String leftTouchSensorName = "tsleft";

    /**
     * Name of the right touch sensor in intake
     */
    public static final String rightTouchSensorName = "tsright";

    /**
     * Name of the LED string
     */
    public static final String LEDName = "LED";

    /**
     * Name of the Linear Slide drive motor
     */
    public static final String linearSlideMotorName = "linear_slide";

    public static final String LeftWebcamName = "Webcam_Left";

    public static final String RightWebcamName = "Webcam_Right";

    public static final String RightSpaceBarName = "tsright";

    public static final String LeftSpaceBarName = "tsleft";

    /**
     * Name of the IMU
     */
    public static final String IMUName = "imu";

    protected static final RevBlinkinLedDriver.BlinkinPattern LEDErrorColor = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
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
     * Provides methods for using the robot LED's
     */
    protected RevBlinkinLedDriver leds;
    /**
     * Provides methods for using the linear slide
     */
    protected LinearSlide slide;

    protected BNO055IMU imu;
    protected DistanceSensor intakeDistanceSensor;

    /**
     * The distance sensor on the front of the robot
     */
    protected DistanceSensor frontDistanceSensor;

    /**
     * Allows control of tape measure turret
     */
    protected TapeMeasureTurret turret;

    /**
     * Provides methods for using the outtake
     */
    protected Outtake outtake;

    protected TouchSensor spaceBar;


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
        } catch (InterruptedException | CancellationException e) {
            RobotLog.d("OpMode Terminated, exiting");
        } catch (Exception | AssertionError e) {
            RobotLog.dd("Exception Was thrown", e, "This exception was thrown and resulted in a early termination of the OpMode");
            RobotLog.setGlobalErrorMsg(MiscUtils.getStackTraceAsString(e)); //Appends more information to the error message
        } catch (NoClassDefFoundError e) {
            throw e;
        } catch (Throwable t) {
            RobotLog.dd("Throwable Was thrown", t, "This Throwable was thrown and resulted in a early termination of the OpMode (And likely everything else)");
            throw t;
        } finally {
            this.cleanup();
        }
    }

    protected void cleanup() {

    }

    protected void initSpaceBar() {
        this.spaceBar = new SpaceBar(hardwareMap, RightSpaceBarName, LeftSpaceBarName);
    }


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if opMode is stopped during execution
     */
    protected void initAll() throws InterruptedException {
        initOdometryServos();
        initLEDS();
        checkStop();
        initLinearSlide();
        initVoltageSensor();
        initSpinner();
        initDistanceSensors();
        initTapeMeasureTurret();
        initOuttake();
        initSpaceBar();
        if (voltageSensor.getVoltage() < 12.5) {
            RobotLog.addGlobalWarningMessage("Voltage reported by internal sensor less than 12.5V");
        }
    }

    /**
     * Initializes the Outtake
     */
    protected void initOuttake() throws InterruptedException {
        outtake = new StateOuttake(hardwareMap, GenericOpModeTemplate.bucketColorSensorName, GenericOpModeTemplate.bucketServoName, true);
        outtake.close();
    }

    public void initTapeMeasureTurret() {
        this.turret = new TapeMeasureTurret(hardwareMap, "tape_measure", "pitch", "yaw");
    }


    public void initDistanceSensors() {
        intakeDistanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");
        frontDistanceSensor = (DistanceSensor) hardwareMap.get("front_distance_sensor");
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
        spinner = new CarouselSpinner(hardwareMap, leftCarouselSpinnerName, rightCarouselSpinnerName);
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
        slide = new LinearSlide(hardwareMap, linearSlideMotorName);
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
