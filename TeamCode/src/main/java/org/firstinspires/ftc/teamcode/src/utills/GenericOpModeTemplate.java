package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.OdometryPodServos;

public abstract class GenericOpModeTemplate extends LinearOpMode {
    public static final String frontRightName = "front_right/vr";
    public static final String frontLeftName = "front_left/vl";
    public static final String backRightName = "back_right/h";
    public static final String backLeftName = "back_left";

    public static final String verticalLeftServoName = "vertical_left_odometry_servo";
    public static final String verticalRightServoName = "vertical_right_odometry_servo";
    public static final String horizontalServoName = "horizontal_odometry_servo";

    public static final String carouselSpinnerName = "cs";

    public static final String bucketServoName = "bucket";
    public static final String intakeMotorName = "intake";
    public static final String bucketColorSensorName = "color_sensor";

    public static final String LEDName = "LED";

    public static final String linearSlideMotorName = "linear_slide";

    public static final String IMUName = "imu";


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

    protected void initAll() throws InterruptedException {
        initOdometryServos();
        initIntake();
        initLEDS();
        checkStop();
        initLinearSlide();
        initVoltageSensor();
        initSpinner();
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
