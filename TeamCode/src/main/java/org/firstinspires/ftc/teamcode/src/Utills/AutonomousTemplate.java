package org.firstinspires.ftc.teamcode.src.Utills;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.OdometryDrivetrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.IMU;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.ContinuousIntake;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;

@Disabled
public abstract class AutonomousTemplate extends LinearOpMode {
    public OdometryPodServos podServos;
    public OdometryDrivetrain driveSystem;
    public CarouselSpinner spinner;
    public OdometryGlobalCoordinatePosition odometry;
    public ContinuousIntake intake;
    public LinearSlide slide;
    public RevBlinkinLedDriver leds;


    public void initAll() throws InterruptedException {
        podServos = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
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

        driveSystem = new OdometryDrivetrain(front_right, front_left, back_right, back_left, telemetry, odometry, this::isStopRequested, this::opModeIsActive);


        spinner = new CarouselSpinner(hardwareMap, "duck_spinner");

        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "slide_motor", s, this::opModeIsActive, this::isStopRequested);
        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        slide.start();
        checkStop();

        intake = new ContinuousIntake(hardwareMap, "intake_motor", "bucketServo");
        intake.setServoUp();

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();

    }

    protected void checkStop() throws InterruptedException {
        if (this.isStopRequested() || Thread.currentThread().isInterrupted()) {
            throw new InterruptedException();
        }
    }

}
