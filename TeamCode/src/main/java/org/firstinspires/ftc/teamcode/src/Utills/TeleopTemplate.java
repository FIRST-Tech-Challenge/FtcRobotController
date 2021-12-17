package org.firstinspires.ftc.teamcode.src.Utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;


@Disabled
public abstract class TeleopTemplate extends LinearOpMode {

    public TeleopDriveTrain driveTrain;
    public CarouselSpinner spinner;
    public OdometryPodServos pod;
    public LinearSlide slide;
    public ContinuousIntake intake;
    public RevBlinkinLedDriver leds;

    public void initAll() {
        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");

        spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        pod = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        pod.raise();

        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "slide_motor", s, this::opModeIsActive, this::isStopRequested);

        intake = new ContinuousIntake(hardwareMap, "intake_motor", "bucketServo", "color_sensor", true);
        intake.setServoDown();

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();
    }
}
