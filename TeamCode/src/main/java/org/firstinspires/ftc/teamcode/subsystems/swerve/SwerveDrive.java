package org.firstinspires.ftc.teamcode.subsystems.swerve;


import android.util.Pair;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;


public class SwerveDrive {
    OpMode OM;
    double theta;
    Gamepad gamepad; // perhaps a set power method would be better here?
    public final double DIST_MULT = 4/3; // Actually traveled/desired dist
    public static double aP = 0.06;
    public static double aI = 0.01;
    public static double aD = 0.0005;
    boolean dontMove;
    boolean reverse;
    public SwerveDriveOdometry odo;
    public ElapsedTime timer;
    public SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveKinematics kinematics; // FTCLIB FOR AUTO
    public static double metersPerTick = (0.061*Math.PI)/770;
    voltageToAngleConstants angleGetter;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    double[] driveSpeeds = new double[4];
    double[] anglePowers = new double[4];
    double[] drivePowers = new double[4];
    public DcMotorEx[] driveMotors = new DcMotorEx[4];
    CRServo[] angleMotors = new CRServo[4];
    int[] lastDriveEncoders = new int[4];
    OptimalAngleCalculator angleFixer;
    double[] angles = new double[4];
    public Pose2d nowPose;
    IMU imu;
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // key = mag, value = direction
    public SwerveDrive(double length, double width, double maxRot, double maxTrans, OpMode opmode, Gamepad GP, HardwareMap hw, String[] encoderNames, String[] driveNames, String[] angleNames, double angleP, double angleI, double angleD) {
        OM = opmode;
        gamepad = GP;
        aP = angleP; //I don't know what I'm doing - owner of the code
        aI = angleI;
        aD = angleD;
        anglePID = new PIDController[]{new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD)};
        angleGetter = new voltageToAngleConstants(opmode, hw, encoderNames);
        vectorGetter = new gamepadToVectors();
        angleFixer = new OptimalAngleCalculator();
        vectorGetter.maxRotationSpeed = maxRot;
        vectorGetter.maxTranslationSpeed = maxTrans;
        vectorGetter.ROBOT_LENGTH = length;
        vectorGetter.ROBOT_WIDTH = width;

        for (int i = 0; i < driveNames.length; i++) {
            driveMotors[i] = (DcMotorEx) hw.get(DcMotor.class, driveNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angleMotors[i] = hw.get(CRServo.class, angleNames[i]);
        }

//        driveMotors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu = hw.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //CHANGE THESE ONCE ORIENTATION IS KNOW
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        // Adjust the orientation parameters to match your robot


        for (int i = 0; i < driveMotors.length; i++) {
            driveSpeeds[i] = getVelocity(driveMotors[i]);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
        }
        targetADPairList.ensureCapacity(4);
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        kinematics = new SwerveDriveKinematics(
                new Translation2d(inchesToMeters(6.5), inchesToMeters(5.5)),
                new Translation2d(inchesToMeters(6.5),inchesToMeters(-5.5)),
                new Translation2d(inchesToMeters(-4.5), inchesToMeters(5.5)),
                new Translation2d(inchesToMeters(-4.5), inchesToMeters(-5.5)));
        odo = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw())), new Pose2d(0, 0, new Rotation2d(0)));
        timer = new ElapsedTime();
        nowPose = new Pose2d();

        for (int i = 0; i < driveMotors.length; i++) {
            if (Math.signum(driveSpeeds[i]) == -1) {
//                if (angles[i] + 180 > 360) {
//                    states[i] = new SwerveModuleState(-1 * driveSpeeds[i], new Rotation2d(angles[i]));
//                } else {
                states[i] = new SwerveModuleState(-1 * driveSpeeds[i], new Rotation2d(Math.toRadians((angles[i] + 180) % 360)));
//                }
            } else {
                states[i] = new SwerveModuleState(driveSpeeds[i], new Rotation2d(Math.toRadians(angles[i])));
            }

        }
        // init the other devices
    }
    public void resetIMU() { imu.resetYaw();}
    public void init_loop () {
        angleGetter.init_loop();
        timer.reset();
    }
    public void updateMagnitudeDirectionPair(double x, double y, double rx, double currentAngle, int m) {
        theta = imu.getRobotYawPitchRollAngles().getYaw();
        if (theta < 0) {theta +=360; }
        else if (theta > 360) {theta -=360; }
            // Get the combined vector from gamepad inputs
        double[] componentsVector = vectorGetter.getCombinedVector(
                theta,
                x,
                y,
                rx,
                gamepadToVectors.Wheel.values()[m]
        );

        // Calculate magnitude and direction
        double magnitude = Math.sqrt(Math.pow(componentsVector[0], 2) + Math.pow(componentsVector[1], 2));

        double direction;
        if (Math.abs(magnitude) > 0.1) {
            direction = Math.toDegrees(Math.atan2(componentsVector[0], componentsVector[1])) + 180;
//            if (direction < 0) {
//                direction += 180;
//            }
            direction = angleFixer.calculateOptimalAngle(currentAngle, direction, m); // perhaps this is where fixes needed?
            dontMove = false;
        } else {
            Pair<Double, Double> pairNoMove = new Pair<>(0.0, targetADPairList.get(m).second);
            targetADPairList.set(m, pairNoMove);
            dontMove = true;
            return;
        }
        // cosine compensation
        magnitude *= Math.abs(Math.cos(Math.toRadians(Math.abs(angles[m]-direction))));
        // Adjust the magnitude if a direction reversal is needed
        if (angleFixer.requiresReversing(m)) {
            reverse = true;
            Pair<Double, Double> pair = new Pair<>(-magnitude, direction);
            targetADPairList.set(m, pair);
        } else {
            reverse = false;
            Pair<Double, Double> pair = new Pair<>(magnitude, direction);
            targetADPairList.set(m, pair);
        }

    }
    public double getVelocity(DcMotorEx motor) {
        // Problems here - it's returning like 1/100 what it shoul
//        double timeChange = timer.nanoseconds() - lastNanoSeconds;
//        double ticksPerSecond;
//        if (Math.abs(timeChange) > 1E-6) {
//            ticksPerSecond = (tickChange) / timeChange;
//        } else {
//            ticksPerSecond = 0;
//        }
        double ticksPerSecond = motor.getVelocity();
        return metersPerTick * ticksPerSecond * DIST_MULT; // quick fix for now , x 100 but core issue is unk
    }
    public void loop(double x, double y, double rx) {
        angles = angleGetter.getBigPulleyAngles();
        angleGetter.loop();
//        OM.telemetry.addData("Angles", angles.length);
//        OM.telemetry.addData("Angle PId", anglePID.length);
//        OM.telemetry.addData("Drive PId", drivePID.length);
//        OM.telemetry.update();

        for (int i = 0; i < angles.length; i++) {
            updateMagnitudeDirectionPair(x, y, rx, angles[i], i);
            // refresh target numbers based on current pose
            // calculate current velocity
            double set;
            double diff;
            double angleOutput;
            if (!dontMove) {
//                anglePID[i].setSetPoint(targetADPairList.get(i).second);
                set = targetADPairList.get(i).second;
            } else {
                set = angles[i];
//                anglePID[i].setSetPoint(angles[i]);
            }
             // TODO: This returns NaN but input is not the problem
            diff = angles[i] - set;
            angleOutput = anglePID[i].calculate(diff, 0);
            double speedOutput = targetADPairList.get(i).first;

//            } else {
//                speedOutput = 0;
//            }


//            // this is returning 0 for some reason
//            // set PID current to current things
            angleMotors[i].setPower(-angleOutput);
            driveMotors[i].setPower(speedOutput);

            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();

            anglePowers[i] = angleOutput;
            drivePowers[i] = speedOutput;
            driveSpeeds[i] = getVelocity(driveMotors[i]);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
//            if (Math.signum(driveSpeeds[i]) == -1) {
//                states[i].speedMetersPerSecond = -1 * driveSpeeds[i];
//                states[i].angle = new Rotation2d(Math.toRadians((angles[i]+ 180 )));
//            } else {
                states[i].speedMetersPerSecond = -driveSpeeds[i];
                states[i].angle = new Rotation2d(Math.toRadians(angles[i]));
//            }
            // reset the last position and time for velocity calcs
        }
        /* I think this isn't updating the odo's pose, but it's doing smth else such
        that calculating things where all are 0 deg etc somehow makes it think diagonally?

        * check the angle fixing code for being wrong - States takes radians
        * unknown angle range?
        * something I did when typing htat comment last night fixed it

        The update method of the odometry class updates the robot position on the field.
        The update method takes in the gyro angle of the robot, along with an array of
        SwerveModulePosition objects. It is important that the order in which you pass the
        SwerveModulePosition objects is the same as the order in which you created the kinematics
         object.
         */
        nowPose = odo.updateWithTime(timer.seconds(), new Rotation2d(Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw())), states);
    }
    public void setPID(double ap, double ai, double ad) {
        for (PIDController pid : anglePID) {
            pid.setPID(ap, ai, ad);
        }
    }
    public void getTelemetry(Telemetry t) {
        t.addData("FLTargAng", targetADPairList.get(0).second);
        t.addData("FRTargAng", targetADPairList.get(1).second);
        t.addData("BLTargAng", targetADPairList.get(2).second);
        t.addData("BRTargAng", targetADPairList.get(3).second);
        t.addData("FLIn", angles[0]);
        t.addData("FRIn", angles[1]);
        t.addData("BLIn", angles[2]);
        t.addData("BRIn", angles[3]);
        t.addData("FLPos", driveMotors[0].getCurrentPosition());
        t.addData("FLAng", anglePowers[0]);
        t.addData("FRAng", anglePowers[1]);
        t.addData("BLAng", anglePowers[2]);
        t.addData("BRAng", anglePowers[3]);
        t.addData("FLDrive", drivePowers[0]);
        t.addData("FRDrive", drivePowers[1]);
        t.addData("BLDrive", drivePowers[2]);
        t.addData("BRDrive", drivePowers[3]);
        t.addData("reverseFL", angleFixer.reverses()[0]);
        t.addData("reverseFR", angleFixer.reverses()[1]);
        t.addData("reverseBL", angleFixer.reverses()[2]);
        t.addData("reverseBR", angleFixer.reverses()[3]);
        t.addData("yaw", theta);
        t.addData("offsetFL", angleGetter.offsets[0]);
        t.addData("offsetFR", angleGetter.offsets[1]);
        t.addData("offsetBL", angleGetter.offsets[2]);
        t.addData("offsetBR", angleGetter.offsets[3]);
        t.addData("FLVelocity", states[0].speedMetersPerSecond);
        t.addData("FRVelocity", states[1].speedMetersPerSecond);
        t.addData("BLVelocity", states[2].speedMetersPerSecond);
        t.addData("BRVelocity", states[3].speedMetersPerSecond);
        t.addData("FLState", states[0].angle.getDegrees());
        t.addData("FRState", states[1].angle.getDegrees());
        t.addData("BLState", states[2].angle.getDegrees());
        t.addData("BRState", states[3].angle.getDegrees());
        t.addData("velRawFL", driveMotors[0].getVelocity());
        t.addData("velRawFR", driveMotors[1].getVelocity());
        t.addData("velRawBL", driveMotors[2].getVelocity());
        t.addData("velRawBR", driveMotors[3].getVelocity());
        t.addData("PoseX", nowPose.getX());
        t.addData("PoseY", nowPose.getY());
        t.update();
    }
    public double inchesToMeters(double inches) {
        return inches/39.37;
    }
}
