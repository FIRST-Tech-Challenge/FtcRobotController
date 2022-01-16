package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.OdometryPodServos;


/**
 * A template for all Teleop opModes, allows easy initialization
 */
@Disabled
public abstract class TeleopTemplate extends LinearOpMode {

    /**
     * Provides methods for driving
     */
    protected TeleopDriveTrain driveTrain;

    /**
     * Provides Carousel Spinner functionality
     */
    protected CarouselSpinner spinner;

    /**
     * Provides methods for lifting and lowering Odometry servos
     */
    protected OdometryPodServos pod;

    /**
     * Provides methods for using the linear slide
     */
    protected LinearSlide slide;

    /**
     * Provides methods for using the intake
     */
    protected ContinuousIntake intake;

    /**
     * Provides methods for using the robot LED's
     */
    protected RevBlinkinLedDriver leds;

    /**
     * Initializes the objects provided by this class
     */
    protected void initAll() {
        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "cs");

        pod = new OdometryPodServos(hardwareMap, "vertical_right_odometry_servo", "vertical_left_odometry_servo", "horizontal_odometry_servo");
        pod.raise();

        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "linear_slide", s, this::opModeIsActive, this::isStopRequested);


        intake = new ContinuousIntake(hardwareMap, "intake", "bucket", "color_sensor", true);
        intake.setServoClosed();

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
    }
}
