package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Blue Observation", group="Robot")
public class AutoBlueObservation extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.0;
    public static final double TURN_SPEED = 0.5;
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

        robotController.distanceDrive(24.186773244895647, 7.125016348901798, DRIVE_SPEED);
        robotController.sleep(3.0);
        robotController.distanceDrive(26.999999999999993, 270.0, DRIVE_SPEED);
        robotController.distanceDrive(24.0, 0.0, DRIVE_SPEED);
        robotController.distanceDrive(12.0, 270.0, DRIVE_SPEED);
        robotController.distanceDrive(42.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(42.0, 0.0, DRIVE_SPEED);
        robotController.distanceDrive(9.000000000000002, 270.0, DRIVE_SPEED);
        robotController.distanceDrive(42.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(42.0, 0.0, DRIVE_SPEED);
        robotController.distanceDrive(5.999999999999998, 270.0, DRIVE_SPEED);
        robotController.distanceDrive(42.0, 180.0, DRIVE_SPEED);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
        gyro.resetYaw();

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT")
        );
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, this);

        telemetry.addData("Status", "Initialized");
    }
}
