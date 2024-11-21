package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="3 Specimen Auto Observation", group="Robot")
public class AutoObservation3Specimen extends LinearOpMode {
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

        // first specimen
        pivot.setTargetPosition(-1375);
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(1230);
        robotController.distanceDrive(28.460498941515414, -18.434948822922024, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        // push two samples into observation zone
        robotController.distanceDrive(30.59411708155671, 101.30993247402023, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(27.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(36.0, 180.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(36.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(8.999999999999986, 90.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(36.0, 180.0, DRIVE_SPEED + 0.75);

        // prep for grabbing second specimen
        intake.hingeToDegree(90);
        pivot.setTargetPosition(-2320);
        robotController.distanceDrive(9.486832980505126, -71.56505117707795, DRIVE_SPEED + 0.75);
        robotController.sleep(2.0);

        // grab the second specimen
        viperSlide.setTargetPosition(400); // probably not correct value
        robotController.distanceDrive(15.0, 180.0, DRIVE_SPEED);
        intake.close();
        robotController.sleep(0.1);
        intake.hingeToDegree(0);
        pivot.setTargetPosition(-1375);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(1320);

        // place the second specimen
        robotController.distanceDrive(45.0, -53.13010235415598, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        // prep to grab third specimen
        intake.hingeToDegree(90);
        pivot.setTargetPosition(-2320);
        robotController.distanceDrive(37.94733192202055, 108.43494882292202, DRIVE_SPEED + 0.75);
        robotController.sleep(2.0);

        // grab the third specimen
        viperSlide.setTargetPosition(400); // again, probably not the right value
        robotController.distanceDrive(15.0, 180.0, DRIVE_SPEED);
        intake.close();
        robotController.sleep(0.1);
        intake.hingeToDegree(0);
        pivot.setTargetPosition(-1375);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(1320);

        // place the third specimen
        robotController.distanceDrive(42.638011210655684, -50.71059313749967, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(700);
        robotController.sleep(0.5);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(10);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");

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
