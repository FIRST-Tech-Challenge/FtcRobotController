package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Red Observation", group="Robot")
public class AutoRedObservation extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.7;
    public static final double TURN_SPEED = 0.8;
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        intake.close();
//        intake.unwhack();
        pivot.setTargetPosition(-1170);
        robotController.distanceDrive(25, -12.528807709151522, DRIVE_SPEED);
        viperSlide.setTargetPosition(1230);
        robotController.sleep(1);
        pivot.setTargetPosition(-1000);
        robotController.sleep(0.5);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
//        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);
        pivot.setTargetPosition(-510);

        robotController.distanceDrive(27.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(27.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(39, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(39, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(39, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(10.0, 0.0, DRIVE_SPEED);
        robotController.distanceDrive(6.0, -90.0, DRIVE_SPEED);
        robotController.turnTo(180.0, TURN_SPEED);
        robotController.sleep(1.5);


        viperSlide.setTargetPosition(1400);
        robotController.distanceDrive(10.0, 180.0, DRIVE_SPEED);
        robotController.sleep(0.5);
        intake.close();
        robotController.sleep(0.2);
        pivot.setTargetPosition(-700);
        robotController.sleep(0.5);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.distanceDrive(39, -82, DRIVE_SPEED);
        robotController.turnTo(0.0, TURN_SPEED);
        robotController.distanceDrive(8.0, -90, DRIVE_SPEED);

        pivot.setTargetPosition(-1400);
        robotController.distanceDrive(9.0, -12.528807709151522, DRIVE_SPEED);
        viperSlide.setTargetPosition(800);
        robotController.sleep(0.5);
        pivot.setTargetPosition(-800);
        robotController.sleep(0.5);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
        intake.open();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        pivot.setTargetPosition(-1375);
        robotController.sleep(10);

//        robotController.distanceDrive(27.0, 90.0, DRIVE_SPEED);
//        robotController.distanceDrive(27.0, -0.0, DRIVE_SPEED);
//        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED);
//        robotController.distanceDrive(39.0, 180.0, DRIVE_SPEED);
//        robotController.distanceDrive(39.0, -0.0, DRIVE_SPEED);
//        robotController.distanceDrive(8.0, 90.0, DRIVE_SPEED);
//        robotController.distanceDrive(39.0, 180.0, DRIVE_SPEED);
//        robotController.distanceDrive(39.0, -0.0, DRIVE_SPEED);
//        robotController.distanceDrive(8.0, 90.0, DRIVE_SPEED);
//        robotController.distanceDrive(39.0, 180.0, DRIVE_SPEED);
//        robotController.distanceDrive(9.0, -0.0, DRIVE_SPEED);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
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
