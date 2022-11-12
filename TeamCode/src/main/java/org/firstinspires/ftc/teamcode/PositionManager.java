package org.firstinspires.ftc.teamcode;
import android.os.Environment;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Objects;


/** Incorporates estimates from multiple sources to create a single positioning estimate
 */
public class PositionManager {
    // Stores the best guess of the robot's position (location + orientation) at any given time. To be accessed by nav methods
    public Position position;
    private Position encoderDelta = new Position(0, 0, 0);

    public EncoderPositioning encoderPositioning;
    public IMUPositioning imuPositioning;

    private static final String LogFile = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/logs/positioning.txt";

    private Telemetry telemetry;

    PositionManager(HardwareMap hardwareMap, Telemetry telemetry) {
        position = new Position();
        this.telemetry = telemetry;
        initSensors(hardwareMap);
    }

    PositionManager(HardwareMap hardwareMap, Telemetry telemetry, double x, double y, double theta){
        position = new Position(x, y, theta);
        this.telemetry = telemetry;
        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        encoderPositioning = new EncoderPositioning();
        imuPositioning = new IMUPositioning(hardwareMap);
    }


    /** Calls all appropriate sensor update methods to get an updated estimate of the Robot's current position
     * @param robot Robot object whose estimate should be updated
     */
    public void updatePosition(Robot robot) {
        imuPositioning.updateRobotOrientation(robot);
        telemetry.addData("theta", robot.positionManager.position.getRotation());
        updateEncoderPosition(encoderPositioning.getDeltaEstimate(robot));
    }


    /** Adds new detected encoder movement change to both a temporary encoderDelta variable and to the overall position attribute
     *
     *  @param subDelta A delta position represented as a vector from the last seen position.
     *                  e.g. delta = Position(1, 1, 0) would mean a movement of 1 inch on all axis with no rotation
     */
    private void updateEncoderPosition(Position subDelta) {
        position.setX(position.getX() + subDelta.getX());
        position.setY(position.getY() + subDelta.getY());

//        position = Position.add(position, subDelta);
//        encoderDelta = Position.add(encoderDelta, subDelta);
    }


    /** To be called from the CV positioning Pipeline; bro a new cv estimate into the position using the encoder deltas
     * @param newPos The CV estimate
     */
    public void updateCvPosition(Position newPos) {
        Position compounded = Position.add(newPos, encoderDelta);

        // NOTE: combine compounded with current position (compounded shouldn't be an unnecessary local)
        position = compounded;
        encoderDelta.reset();
    }
}

/** Estimates the robot's position based on the encoders on the drivetrain's motors. This will require a baseline
 *  position to add onto.
 */
class EncoderPositioning {
    static int ENCODER_COUNTS_PER_ROTATION = 280;
    static double MAGICAL_FACTOR = (12.566 / 2) * 0.449 * 1.2;
    static double MAGICAL_RATIO = MAGICAL_FACTOR / ENCODER_COUNTS_PER_ROTATION;

    static HashMap<RobotConfig.DriveMotors, Double> RollerAngles = new HashMap<RobotConfig.DriveMotors, Double>() {{
        put(RobotConfig.DriveMotors.FRONT_RIGHT, Math.PI / 4.d);
        put(RobotConfig.DriveMotors.FRONT_LEFT, 3 * Math.PI / 4.d);
        put(RobotConfig.DriveMotors.REAR_LEFT, Math.PI / 4.d);
        put(RobotConfig.DriveMotors.REAR_RIGHT, 3 * Math.PI / 4.d);
    }};


    static HashMap<RobotConfig.DriveMotors, String> WheelNames = new HashMap<RobotConfig.DriveMotors, String>() {{
        put(RobotConfig.DriveMotors.FRONT_RIGHT, "FrontRight");
        put(RobotConfig.DriveMotors.FRONT_LEFT, "FrontLeft");
        put(RobotConfig.DriveMotors.REAR_LEFT, "RearLeft");
        put(RobotConfig.DriveMotors.REAR_RIGHT, "RearRight");
    }};

    /** Checks the robot's encoders to get an estimate of the distance and direction traveled.
     *  Read encoder values and use them to create a Position that represents the robot's movement relative to the last time the encoders were read.
     *  Reset encoder values to zero.
     *  Call submitEstimate
     *  @return a position in the form of a vector from the origin that can be added to an existing measurement
     */
    public Position getDeltaEstimate(Robot robot) {
        double theta = robot.positionManager.position.getRotation();
        double deltaPSumX = 0.0d, deltaPSumY = 0.0d;

        for (HashMap.Entry<RobotConfig.DriveMotors, Double> rollerAngle : RollerAngles.entrySet()) {
            int encoderCounts = Objects.requireNonNull(robot.driveMotors.get(rollerAngle.getKey())).getCurrentPosition();
            double force = rollerAngle.getValue();

            deltaPSumX += -encoderCounts * ((Math.cos(theta) * Math.cos(force)) - (Math.sin(theta) * Math.sin(force))) / 2.0;
            deltaPSumY += -encoderCounts * ((Math.sin(theta) * Math.cos(force)) + (Math.cos(theta) * Math.sin(force))) / 2.0;
        }

        resetEncoders(robot);
        return new Position(MAGICAL_RATIO * deltaPSumX, MAGICAL_RATIO * deltaPSumY, 0.0);


//        double ADEncCount = robot.frontLeftDrive.getCurrentPosition() + robot.rearRightDrive.getCurrentPosition();
//        double BCEncCount = robot.rearLeftDrive.getCurrentPosition() + robot.frontRightDrive.getCurrentPosition();
//
//        double ADangle = Math.acos((2 * ADEncCount + Math.sqrt(8 - 4 * (ADEncCount * ADEncCount))) / 4);
//        double BCangle = Math.acos((2 * BCEncCount + Math.sqrt(8 - 4 * (BCEncCount * BCEncCount))) / 4);
//
//        double secondXcor = BCEncCount * Math.cos(BCangle);
//        double secondYcor = BCEncCount * Math.sin(BCangle);
//
//        double thirdXcor = secondXcor - ADEncCount * Math.cos(ADangle);
//        double thirdYcor = secondYcor + ADEncCount * Math.sin(ADangle);
//        double orientation = Math.atan(thirdXcor / thirdYcor);

//        submitEstimate(robot, new Position(thirdXcor, thirdYcor, 0.0));
    }


    /** Resets the encoder values to zero.
     */
    private void resetEncoders(Robot robot) {
        for (RobotConfig.DriveMotors motor : RobotConfig.DriveMotors.values()) {
            // We need to figure out which one of these is actually resetting (if both; we want only RESET)
            // Objects.requireNonNull(robot.driveMotors.get(motor)).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Objects.requireNonNull(robot.driveMotors.get(motor)).setMode(DcMotor.RunMode.RESET_ENCODERS);
            Objects.requireNonNull(robot.driveMotors.get(motor)).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}


class IMUPositioning {
    static private BNO055IMU imu;
    static private BNO055IMU.Parameters parameters;


    IMUPositioning(HardwareMap hardwareMap) {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }


    public static void Initialize(OpMode opMode) {
        imu.initialize(parameters);

//        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
//            opMode.sleep(50);
//            opMode.idle();
//        }

        while (!imu.isGyroCalibrated()) {
        }
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    void updateRobotOrientation(Robot robot) {
        robot.positionManager.position.setRotation(getAngle());
    }
}