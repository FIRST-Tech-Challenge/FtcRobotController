//
//package org.firstinspires.ftc.teamcode.teleop;
//
//import static java.lang.Math.cos;
//import static java.lang.Math.sin;
//
//import com.acmerobotics.roadrunner.MecanumKinematics;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//
//@TeleOp(name="RR in tele", group="Linear OpMode")
////@Disabled
//public class FieldCentricMess extends LinearOpMode {
//
//    // Declare OpMode members for each of the 4 motors.
//    private ElapsedTime runtime = new ElapsedTime();
//    protected MecanumDrive drive;
//    private double  yMultiplier = 1;
//    private IMU imu;
//    private Pose2d startingPos = new Pose2d(0,0,Math.toRadians(90));
//
//    public static double VX_WEIGHT = 1;
//    public static double VY_WEIGHT = 1;
//    public static double OMEGA_WEIGHT = 1;
//    private double lateralMultiplier= 1.0;
//
//    Gamepad currentGamepad1 = new Gamepad();
//    Gamepad currentGamepad2 = new Gamepad();
//
//    Gamepad previousGamepad1 = new Gamepad();
//    Gamepad previousGamepad2 = new Gamepad();
//
//    @Override
//    public void runOpMode() {
//        // region Hardware Initialization
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters myIMUparameters;
//
//        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//
//        // Initialize IMU using Parameters
//        imu.initialize(myIMUparameters);
//        imu.resetYaw();
//
//        drive = new MecanumDrive(hardwareMap, startingPos);
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
//
//            Pose2d poseEstimate = drive.pose;
//
//            // Create a vector from the gamepad x/y inputs
//            // Then, rotate that vector by the inverse of that heading
//            YawPitchRollAngles robotOrientation;
//            robotOrientation = imu.getRobotYawPitchRollAngles();
//            Vector2d input = new Vector2d(-currentGamepad1.left_stick_x * yMultiplier,
//                    currentGamepad1.left_stick_y * yMultiplier);
//                    //.times(Math.toRadians(poseEstimate.heading.toDouble()));
//                    //highly sus code above
//                    //this is replica of the rotate code from last year
//            Vector2d rotated = rotatedVector(drive.pose.heading.toDouble());
//
////            if (currentGamepad1.left_bumper) {
////                input = new Vector2d(-currentGamepad1.left_stick_x * yMultiplier / 3,
////                        currentGamepad1.left_stick_y * yMultiplier / 3);
////                rotated = rotatedVector(drive.pose.heading.toDouble());
////                //.rotated(-robotOrientation.getYaw(AngleUnit.RADIANS));
////
////                // Pass in the rotated input + right stick value for rotation
////                // Rotation is not part of the rotated input thus must be passed in separately
////                double heading = drive.pose.heading.toDouble();
////                drive.setDrivePowers();
////            } else {
//
//                // Pass in the rotated input + right stick value for rotation
//                // Rotation is not part of the rotated input thus must be passed in separately
//
//                drive.setDrivePowers(new Pose2d(input.x, input.y,input.y, rotated.) -currentGamepad1.right_stick_x);
//            //}
//
//            if (currentGamepad1.options && !previousGamepad1.options) {
//                if (yMultiplier == 1) {
//                    yMultiplier = -1;
//                } else {
//                    yMultiplier = 1;
//                }
//            }
//
//            // Update everything. Odometry. Etc.
//            drive.updatePoseEstimate();
//
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.update();
//        }
//    }
//    private Vector2d rotatedVector(double angle){
//        double x = -currentGamepad1.left_stick_x * yMultiplier;
//        double y = currentGamepad1.left_stick_y * yMultiplier;
//        double newX = x * cos(angle) - y * sin(angle);
//        double newY = x * sin(angle) + y * cos(angle);
//        return new Vector2d(newX,newY);
//    }
//    public void setWeightedDrivePower(Pose2d drivePower) {
//        Pose2d vel = drivePower;
//
//        if (drivePower.heading.toDouble() > 1) {
//            // re-normalize the powers according to the weights
//            double denom = VX_WEIGHT * Math.abs(drivePower.position.x)
//                    + VY_WEIGHT * Math.abs(drivePower.position.y)
//                    + OMEGA_WEIGHT * Math.abs(drivePower.heading.toDouble());
//
//            vel = new Pose2d(
//                    VX_WEIGHT * drivePower.position.x/denom,
//                    VY_WEIGHT * drivePower.position.y/denom,
//                    OMEGA_WEIGHT * drivePower.heading.toDouble()
//            )
//                    //div(denom);
//        }
//        MecanumKinematics powers = new MecanumKinematics.WheelVelocities(
//                drivePower,
//                1.0,
//                1.0,
//                lateralMultiplier
//        );
//        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
//        setDrivePower(vel);
//    }
//}
