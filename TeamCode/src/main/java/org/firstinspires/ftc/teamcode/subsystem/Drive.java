package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Drive extends SubsystemBase {
    public Motor leftFront;
    public Motor rightFront;
    public Motor leftBack;
    public Motor rightBack;
    public IMU imu;
    private final Telemetry telemetry;
    private final MecanumDrive mecanumDrive;
    MecanumDriveOdometry mecanumDriveOdometry;
    MecanumDriveKinematics kinematics;
    Pose2d m_pose;
    private ElapsedTime runtime = new ElapsedTime();

    // sets drive speed states
    public final double SPRINT = 0.9;
    public final double NORMAL = 0.6;
    public final double SLOW = 0.4;

    // current state
    public double state = NORMAL;
    List<LynxModule> allHubs;
    final int TEST_CYCLES    = 500;
    int cycles;
    private long      e1, e2, e3, e4; // Encoder Values
    private double    v1, v2, v3, v4; // Velocities
    // Cycle Times
    double t1 = 0;
    double t2 = 0;
    double t3 = 0;

    /**
     * Initializes drivetrain  and IMU
     * @param hardwareMap hardware map from teleOp for motor setup
     */
    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        allHubs = hardwareMap.getAll(LynxModule.class);
        leftFront = new Motor(hardwareMap, "motorTest1", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hardwareMap, "motorTest2", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hardwareMap, "motorTest0", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hardwareMap, "motorTest3", Motor.GoBILDA.RPM_312);

        // access motors, reverses directions and sets braking behaviour
//        leftFront.motor.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.motor.setDirection(DcMotor.Direction.REVERSE);
//        leftBack.motor.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.motor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map and initialize
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        resetEncoders();
        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        Translation2d frontLeftLocation = new Translation2d(0.11, 0.11);
        Translation2d frontRightLocation = new Translation2d(0.11, -0.11);
        Translation2d backLeftLocation = new Translation2d(-0.11, 0.11);
        Translation2d backRightLocation = new Translation2d(-0.11, -0.11);

        kinematics = new MecanumDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        );
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imuAngle());

        mecanumDriveOdometry = new MecanumDriveOdometry(
                kinematics,gyroAngle,
                new Pose2d(0,0, new Rotation2d())
        );
        runtime.reset();
//        testBulkRed(hardwareMap);
    }

    /**
     * Resets motor encoder velues
     */
    public void resetEncoders() {
        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
//                leftFront.encoder.getRate(), rightFront.encoder.getRate(),
//                leftBack.encoder.getRate(), rightBack.encoder.getRate()
                rightBack.encoder.getRate(), rightBack.encoder.getRate(),
                rightBack.encoder.getRate(), rightBack.encoder.getRate()
        );

        // Get my gyro angle.
        Rotation2d gyroAngle = Rotation2d.fromDegrees(imuAngle());

        // Update the pose
        m_pose = mecanumDriveOdometry.updateWithTime(runtime.seconds(), gyroAngle, wheelSpeeds);

        telemetry.addData("RightRear3 Position", rightBackPos());
        telemetry.addData("Motor Power", rightBackPower());
        telemetry.addData("IMU Value", imuAngle());
        telemetry.addData("Pose x", m_pose.getX());//39.5cm-981/71-1717
        telemetry.addData("Pose y", m_pose.getY());
        telemetry.addData("Run time", runtime);
        telemetry.update();
    }

    public void mecanumFieldDrive(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        mecanumDrive.driveFieldCentric(-strafeSpeed, forwardSpeed, -turnSpeed, imuAngle(), squareInputs);
    }
    public void mecanumCentricDrive(double strafeRightPositive, double forwardPositive, double rotateClockwisePositive, boolean squareInputs) {
        mecanumDrive.driveRobotCentric(-strafeRightPositive, forwardPositive, -rotateClockwisePositive, squareInputs);
    }
    public int rightBackPos() {
        return rightBack.getCurrentPosition();
    }
    public double rightBackPower() {
        return rightBack.get();
    }
    public double rightBackVelocity() {
        return rightBack.getCorrectedVelocity();
    }

    /**
     * Resets IMU heading
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Returns current heading
     * @return Current IMU heading
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double imuAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Manually set speed multiplier
     * @param speed Speed multiplier
     */
    public void setSpeed(double speed) {
        state = speed;
    }

    public void testBulkRed(HardwareMap hardwareMap) {
        displayCycleTimes("Test 1 of 3 (Wait for completion)");

        runtime.reset();
        cycles = 0;
        while ((cycles++ < TEST_CYCLES)) {
            e1 = leftFront.getCurrentPosition();
            e2 = leftBack.getCurrentPosition();
            e3 = rightFront.getCurrentPosition();
            e4 = rightBack.getCurrentPosition();

            v1 = leftFront.getCorrectedVelocity();
            v2 = leftBack.getCorrectedVelocity();
            v3 = rightFront.getCorrectedVelocity();
            v4 = rightBack.getCorrectedVelocity();

            // Put Control loop action code here.

        }
        // calculate the average cycle time.
        t1 = runtime.milliseconds() / cycles;
        displayCycleTimes("Test 2 of 3 (Wait for completion)");

        // --------------------------------------------------------------------------------------
        // Run test cycles using AUTO cache mode
        // In this mode, only one bulk read is done per cycle, UNLESS you read a specific encoder/velocity item AGAIN in that cycle.
        // --------------------------------------------------------------------------------------

        // Important Step 3: Option A. Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        runtime.reset();
        cycles = 0;
        while ((cycles++ < TEST_CYCLES)) {
            e1 = leftFront.getCurrentPosition();  // Uses 1 bulk-read for all 4 encoder/velocity reads,
            e2 = leftBack.getCurrentPosition();  // but don't do any `get` operations more than once per cycle.
            e3 = rightFront.getCurrentPosition();
            e4 = rightBack.getCurrentPosition();

            v1 = leftFront.getCorrectedVelocity();
            v2 = leftBack.getCorrectedVelocity();
            v3 = rightFront.getCorrectedVelocity();
            v4 = rightBack.getCorrectedVelocity();

            // Put Control loop action code here.

        }
        // calculate the average cycle time.
        t2 = runtime.milliseconds() / cycles;
        displayCycleTimes("Test 3 of 3 (Wait for completion)");

        // --------------------------------------------------------------------------------------
        // Run test cycles using MANUAL cache mode
        // In this mode, only one block read is done each control cycle.
        // This is the MOST efficient method, but it does require that the cache is cleared manually each control cycle.
        // --------------------------------------------------------------------------------------

        // Important Step 3: Option B. Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        runtime.reset();
        cycles = 0;
        while ((cycles++ < TEST_CYCLES)) {

            // Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            e1 = leftFront.getCurrentPosition();   // Uses 1 bulk-read to obtain ALL the motor data
            e2 = leftBack.getCurrentPosition();   // There is no penalty for doing more `get` operations in this cycle,
            e3 = rightFront.getCurrentPosition();   // but they will return the same data.
            e4 = rightBack.getCurrentPosition();

            v1 = leftFront.getCorrectedVelocity();
            v2 = leftBack.getCorrectedVelocity();
            v3 = rightFront.getCorrectedVelocity();
            v4 = rightBack.getCorrectedVelocity();

            // Put Control loop action code here.

        }
        // calculate the average cycle time.
        t3 = runtime.milliseconds() / cycles;
        displayCycleTimes("Complete");
    }
    // Display three comparison times.
    void displayCycleTimes(String status) {
        telemetry.addData("Testing", status);
        telemetry.addData("Cache = OFF",    "%5.1f mS/cycle", t1);
        telemetry.addData("Cache = AUTO",   "%5.1f mS/cycle", t2);
        telemetry.addData("Cache = MANUAL", "%5.1f mS/cycle", t3);
    }
}
