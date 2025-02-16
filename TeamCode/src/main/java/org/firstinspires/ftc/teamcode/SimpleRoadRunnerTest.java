package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Simple RoadRunner Test")
public class SimpleRoadRunnerTest extends LinearOpMode {
    // Hardware
    private DcMotorEx xEncoder, yEncoder;  // One encoder for each axis
    private BNO055IMU imu;
    
    // RoadRunner Drive Constants
    public static final double TICKS_PER_REV = 8192;  // REV Through Bore Encoder
    public static final double WHEEL_RADIUS = 0.75;  // inches
    public static final double GEAR_RATIO = 1.0;     // output (wheel) speed / input (encoder) speed
    
    // Tracking wheel locations relative to robot center
    public static final double X_OFFSET = 2.0;  // X distance of the parallel wheel from robot center
    public static final double Y_OFFSET = 2.0;  // Y distance of the perpendicular wheel from robot center
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        // Create RoadRunner drive
        RoadRunnerMecanumDrive drive = new RoadRunnerMecanumDrive(hardwareMap);
        
        // Create trajectory
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
            .forward(12.0)  // Move forward 1 foot (12 inches)
            .build();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        // Follow trajectory
        drive.followTrajectory(trajectory);
        
        // Turn 90 degrees
        drive.turn(Math.toRadians(90));
        
        while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
            // Update drive
            drive.update();
            
            // Display position info
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("X Encoder", xEncoder.getCurrentPosition());
            telemetry.addData("Y Encoder", yEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
    
    private void initHardware() {
        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // Initialize encoders
        xEncoder = hardwareMap.get(DcMotorEx.class, "x_encoder");  // Parallel to robot's X axis
        yEncoder = hardwareMap.get(DcMotorEx.class, "y_encoder");  // Parallel to robot's Y axis
        
        // Reverse encoders if needed (adjust based on your setup)
        xEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set encoder modes
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
} 