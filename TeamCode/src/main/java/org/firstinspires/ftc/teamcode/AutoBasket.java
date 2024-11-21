package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name="Auto Basket", group="Robot")
public class AutoBasket extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.7;
    public static final double TURN_SPEED = 0.8;
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public static double moveSpeed = 1.5;
    public static double distance = 48;
    public static double sleepTime = 100;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();

        // place the preloaded specimen
        pivot.setTargetPosition(-1375);
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(1230);
        robotController.distanceDrive(27.65863337187866, 12.528807709151465, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        // grab first neutral sample
        pivot.setTargetPosition(-430);
        robotController.distanceDrive(40.024992192378996, -102.99461679191654, DRIVE_SPEED + 0.75);
        pivot.setTargetPosition(-200);
        robotController.sleep(0.1);
        intake.close();

        // place first neutral sample in high basket
        pivot.setTargetPosition(-1375);
        intake.hingeToDegree(180);
        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
        robotController.distanceDrive(18.973665961010276, -161.56505117707798, DRIVE_SPEED);
        robotController.turnTo(315.0, TURN_SPEED);
        intake.largeOpen();

        // grab second neutral sample
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        pivot.setTargetPosition(-430);
        robotController.distanceDrive(18.973665961010276, -18.434948822922024, DRIVE_SPEED + 0.75);
        robotController.turnTo(0.0, TURN_SPEED);
        pivot.setTargetPosition(-200);
        robotController.sleep(0.1);
        intake.close();

        // place second neutral sample in high basket
        pivot.setTargetPosition(-1375);
        intake.hingeToDegree(180);
        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
        robotController.distanceDrive(18.973665961010276, 161.56505117707798, DRIVE_SPEED);
        robotController.turnTo(315.0, TURN_SPEED);
        intake.open();

        // grab third neutral sample (trickier because its against the wall
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(100);
        pivot.setTargetPosition(-430);
        robotController.distanceDrive(24.186773244895647, -29.744881296942225, DRIVE_SPEED + 0.75);
        robotController.turnTo(45.0, TURN_SPEED);
        intake.setWristDegree(-45);
        pivot.setTargetPosition(-200);
        robotController.sleep(0.1);
        intake.close();

        // place third neutral sample in high basket
        pivot.setTargetPosition(-430);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.1);
        pivot.setTargetPosition(-1375);
        intake.hingeToDegree(180);
        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
        robotController.distanceDrive(24.186773244895647, 150.25511870305778, DRIVE_SPEED);
        robotController.turnTo(315.0, TURN_SPEED);
        intake.largeOpen();

        // prep for teleop
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT")
        );
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor,this);

        telemetry.addData("Status", "Initialized");
    }
}
