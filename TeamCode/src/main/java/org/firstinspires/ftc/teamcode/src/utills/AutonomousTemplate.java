package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryDrivetrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.IMU;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.OdometryPodServos;

/**
 * A template for Autonomous OpModes, allows for easy initialization
 */
@Disabled
public abstract class AutonomousTemplate extends LinearOpMode {
    /**
     * Provides methods for lifting and lowering Odometry servos
     */
    protected OdometryPodServos podServos;

    /**
     * Provides methods to use Odometry and associated drive methods
     */
    protected OdometryDrivetrain driveSystem;

    /**
     * Provides Carousel Spinner functionality
     */
    protected CarouselSpinner spinner;

    /**
     * Provides Odometry Methods, Also held by OdometryDrivetrain driveSystem
     */
    protected OdometryGlobalCoordinatePosition odometry;

    /**
     * Provides methods for using the intake
     */
    protected ContinuousIntake intake;

    /**
     * Provides methods for using the linear slide
     */
    protected LinearSlide slide;

    /**
     * Provides methods for using the robot LED's
     */
    protected RevBlinkinLedDriver leds;

    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if the OpMode is stopped during function execution
     */
    protected void initAll() throws InterruptedException {
        podServos = new OdometryPodServos(hardwareMap, "vertical_right_odometry_servo", "vertical_left_odometry_servo", "horizontal_odometry_servo");
        podServos.lower();


        DcMotor front_right = hardwareMap.dcMotor.get("front_right/vr");
        DcMotor front_left = hardwareMap.dcMotor.get("front_left/vl");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right/h");
        checkStop();

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder
        //verticalEncoderLeft, verticalEncoderRight, horizontalEncoder
        odometry = new OdometryGlobalCoordinatePosition(front_left, front_right, back_right, 25, this::opModeIsActive, this::isStopRequested);
        checkStop();
        IMU imu = new IMU(hardwareMap, "imu");
        checkStop();
        odometry.setImu(imu.getImu());


        odometry.reverseLeftEncoder();
        odometry.start();
        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);

        driveSystem = new OdometryDrivetrain(front_right, front_left, back_right, back_left, telemetry, odometry, this::isStopRequested, this::opModeIsActive, s);


        spinner = new CarouselSpinner(hardwareMap, "cs");


        slide = new LinearSlide(hardwareMap, "linear_slide", s, this::opModeIsActive, this::isStopRequested);
        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        slide.start();
        checkStop();

        intake = new ContinuousIntake(hardwareMap, "intake", "bucket", "distance_sensor", "color_sensor", true);
        intake.setServoClosed();

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();

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
