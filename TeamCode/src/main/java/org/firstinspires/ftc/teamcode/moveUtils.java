package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class moveUtils {

    // Things that need to be imported
    private static DcMotor LF = null;
    private static DcMotor RF = null;
    private static DcMotor LB = null;
    private static DcMotor RB = null;
    private static CRServo spinner = null;
    private static BNO055IMU imu;
    private static float desiredHeading;

    private static Orientation lastAngles = new Orientation();

    // Things specific to this class
    private static final float TURN_SPEED_HIGH = 1f;
    private static final float TURN_SPEED_LOW = 0.2f; //original = 0.15f
    private static final float TURN_HIGH_ANGLE = 45.0f;
    private static final float TURN_LOW_ANGLE = 5.0f; //changed = 3.0f
    private static float globalAngle = 0f;
    private static double rotation;

    static final float EncoderTicks = 537.6f;
    static final float WHEEL_DIAMETER_INCHES = 4.0f;
    static final float REVS_PER_INCH_MOD = 50f/72f;
    static final float COUNTS_PER_INCH = (EncoderTicks * REVS_PER_INCH_MOD) / (3.1416f * WHEEL_DIAMETER_INCHES);
    static final float SCALE_ADJUST_FWD = 5.0f;

    static final float STRAFE_MOD = 36f;
    static final float MAX_STRAFE_SPEED = 1.0f;
    private static SampleMecanumDrive drive;
    private static actuatorUtils utils;



    public static void initialize(SampleMecanumDrive drive, actuatorUtils utils){
        moveUtils.drive = drive;
        moveUtils.utils = utils;
    }
    public static void driveSeq (double x, double y, double heading){
        Pose2d pose = new Pose2d(x, y, Math.toRadians(heading));
        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
        drive.followTrajectorySequence(seq);
    }
   /* public static void driveToBoard(double x, double y, double heading) throws InterruptedException{
        utils.elbowBoard();
        utils.armBoard();
        sleep(1000);
        moveUtils.driveSeq(x, y, heading);
        sleep(500);
        utils.gripperOpen();
        sleep(1000);
    }
    public static void driveFromBoard(double x, double y, double heading) throws InterruptedException{
        moveUtils.driveSeq(x,y,heading);
        utils.noArmBoard();
        utils.noElbowBoard();
        sleep(1000);
        utils.gripperClose();
    }
*/



}
