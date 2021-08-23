package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.drivetrain.FourWheelDrive;
import org.firstinspires.ftc.teamcode.measure.Imu;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Logger;

public class AppContext {
    private static final AppContext instance = new AppContext();

    private LinearOpMode opMode;
    private DriveTrain driveTrain;
    private Imu imu;

    private AppContext() {
    }

    public static AppContext getInstance() {
        return instance;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        Logger logger = Logger.getInstance();
        logger.setTelemetry(opMode.telemetry);

        //IMU Initialization
        BNO055IMU bno055IMU = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu = new Imu(bno055IMU);
        imu.init();

        DcMotor frontLeft = opMode.hardwareMap.dcMotor.get(Constants.WHEEL_NAME.FRONT_LEFT.name());
        DcMotor frontRight = opMode.hardwareMap.dcMotor.get(Constants.WHEEL_NAME.FRONT_RIGHT.name());
        DcMotor backLeft = opMode.hardwareMap.dcMotor.get(Constants.WHEEL_NAME.BACK_LEFT.name());
        DcMotor backRight = opMode.hardwareMap.dcMotor.get(Constants.WHEEL_NAME.BACK_RIGHT.name());

        FourWheelDrive fourWheelDrive = new FourWheelDrive(frontLeft, frontRight, backLeft, backRight);
        fourWheelDrive.setImu(imu);
        fourWheelDrive.init();
        fourWheelDrive.reset();

        this.driveTrain = fourWheelDrive;

        opMode.waitForStart();
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Imu getImu() {
        return imu;
    }

    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }

    public boolean isOpModeActive() {
        return opMode.opModeIsActive();
    }
}
