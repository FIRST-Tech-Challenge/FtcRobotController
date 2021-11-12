package org.firstinspires.ftc.teamcode.src.Utills;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.OdometryDrivetrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.IMU;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;

@Disabled
public class AutonomousTemplate extends LinearOpMode {
    public OdometryPodServos podServos;
    public OdometryDrivetrain driveSystem;
    public CarouselSpinner spinner;
    public OdometryGlobalCoordinatePosition odometry;
    public LinearSlide slide;

    public void initAll() {
        podServos = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        podServos.lower();


        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right");

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


        //front_left, front_right, back_right
        odometry = new OdometryGlobalCoordinatePosition(front_left, front_right, back_right, 25);
        IMU imu = new IMU(hardwareMap, "imu");
        odometry.setImu(imu.getImu());


        odometry.reverseLeftEncoder();
        Thread positionThread = new Thread(odometry);
        positionThread.start();

        driveSystem = new OdometryDrivetrain(front_right, front_left, back_right, back_left, telemetry, odometry, new Executable<Boolean>() {
            @Override
            public Boolean call() {
                return isStopRequested();
            }
        }, new Executable<Boolean>() {
            @Override
            public Boolean call() {
                return opModeIsActive();
            }
        });


        spinner = new CarouselSpinner(hardwareMap, "duck_spinner"
        );

        slide = new LinearSlide(hardwareMap, "slide_motor");

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
