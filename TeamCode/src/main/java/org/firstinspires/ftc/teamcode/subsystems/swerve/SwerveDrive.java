package org.firstinspires.ftc.teamcode.subsystems.swerve;


import android.util.Pair;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static double aP, aI, aD, dP, dI, dD;
    boolean dontMove;
    boolean reverse;
    public SwerveDriveOdometry odo;
    public ElapsedTime timer;
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveKinematics kinematics; // FTCLIB FOR AUTO
    public static double metersPerTick = 0.0001297;
    voltageToAngleConstants angleGetter;
    gamepadToVectors vectorGetter;
    PIDController[] anglePID;
    PIDController[] drivePID;
    double[] driveSpeeds = new double[4];
    double[] anglePowers = new double[4];
    double[] drivePowers = new double[4];
    DcMotor[] driveMotors = new DcMotor[4];
    CRServo[] angleMotors = new CRServo[4];
    int[] lastDriveEncoders = new int[4];
    ElapsedTime motorTimer;
    OptimalAngleCalculator angleFixer;
    double[] angles = new double[4];
    IMU imu;
    ArrayList<Pair<Double,Double>> targetADPairList = new ArrayList<>(4); // key = mag, value = direction
    public SwerveDrive(double length, double width, double maxRot, double maxTrans, OpMode opmode, Gamepad GP, HardwareMap hw, String[] encoderNames, String[] driveNames, String[] angleNames, double angleP, double angleI, double angleD, double driveP, double driveI, double driveD) {
        OM = opmode;
        gamepad = GP;
        aP = angleP; //I don't know what I'm doing - owner of the code
        aI = angleI;
        aD = angleD;
        dP = driveP;
        dI = driveI;
        dD = driveD;
        anglePID = new PIDController[]{new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD), new PIDController(aP, aI, aD)};
        drivePID = new PIDController[]{new PIDController(dP, dI, dD), new PIDController(dP, dI, dD), new PIDController(dP, dI, dD), new PIDController(dP, dI, dD)};
        angleGetter = new voltageToAngleConstants(opmode, hw, encoderNames);
        vectorGetter = new gamepadToVectors();
        angleFixer = new OptimalAngleCalculator();
        vectorGetter.maxRotationSpeed = maxRot;
        vectorGetter.maxTranslationSpeed = maxTrans;
        vectorGetter.ROBOT_LENGTH = length;
        vectorGetter.ROBOT_WIDTH = width;

        driveMotors[0] = hw.get(DcMotor.class, driveNames[0]);
        driveMotors[1] = hw.get(DcMotor.class, driveNames[1]);
        driveMotors[2] = hw.get(DcMotor.class, driveNames[2]);
        driveMotors[3] = hw.get(DcMotor.class, driveNames[3]);
        angleMotors[0] = hw.get(CRServo.class, angleNames[0]);
        angleMotors[1] = hw.get(CRServo.class, angleNames[1]);
        angleMotors[2] = hw.get(CRServo.class, angleNames[2]);
        angleMotors[3] = hw.get(CRServo.class, angleNames[3]);
//        driveMotors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu = hw.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //CHANGE THESE ONCE ORIENTATION IS KNOW
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        // Adjust the orientation parameters to match your robot

        motorTimer = new ElapsedTime();
        for (int i = 0; i < driveMotors.length; i++) {
            driveSpeeds[i] = getVelocity(driveMotors[i].getCurrentPosition()- lastDriveEncoders[i], motorTimer);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();
        }
        targetADPairList.ensureCapacity(4);
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        targetADPairList.add(new Pair<>(0.0, 0.0));
        kinematics = new SwerveDriveKinematics(
                new Translation2d(-0.22, 0.22),
                new Translation2d(0.22, 0.22),
                new Translation2d(-0.22, -0.22),
                new Translation2d(0.22, -0.22));
        odo = new SwerveDriveOdometry(kinematics, new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw()));
        timer = new ElapsedTime();

        for (int i = 0; i < driveMotors.length; i++) {
            states[i] = new SwerveModuleState(driveSpeeds[i], new Rotation2d(angles[i]));
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
    public double getVelocity(int tickChange, ElapsedTime timer) {
        double timeChange = timer.seconds();
        double ticksPerSecond = tickChange/timeChange;
        return metersPerTick * ticksPerSecond;
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
            if (!dontMove) {
                anglePID[i].setSetPoint(targetADPairList.get(i).second);
            } else {
                anglePID[i].setSetPoint(angles[i]);
            }
//            drivePID[i].setSetPoint(targetADPairList.get(i).first);
            // set PID target to be the ones calculated earlier
            double speedOutput;

        // here be PIDs
        // control motors, setpower
            double angleOutput = anglePID[i].calculate(angles[i]); // TODO: This returns NaN but input is not the problem
//            if (Math.abs(angles[i] - targetADPairList.get(i).second) < 15) { // if within 15 deg of targ angle
//            if (angleFixer.requiresReversing) {
            speedOutput = targetADPairList.get(i).first;

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
            states[i] = new SwerveModuleState(driveSpeeds[i], new Rotation2d(angles[i]));
            driveSpeeds[i] = getVelocity(driveMotors[i].getCurrentPosition()- lastDriveEncoders[i], motorTimer);
            lastDriveEncoders[i] = driveMotors[i].getCurrentPosition();

            // reset the last position and time for velocity calcs
        }
        odo.updateWithTime(timer.seconds(), new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw()), states);
    }
    public void setPID(double ap, double ai, double ad, double dp, double di, double dd) {
        for (PIDController pid : anglePID) {
            pid.setPID(ap, ai, ad);
        }
        for (PIDController pid : drivePID) {
            pid.setPID(dp, di, dd);
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
        t.addData("FLVelocity", driveSpeeds[0]);
        t.addData("FRVelocity", driveSpeeds[1]);
        t.addData("BLVelocity", driveSpeeds[2]);
        t.addData("BRVelocity", driveSpeeds[3]);
        t.update();
    }
}
