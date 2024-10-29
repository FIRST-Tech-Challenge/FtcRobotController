// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.controller.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.util.Units;
import org.firstinspires.ftc.teamcode.Utils;

public class Swerve {

  private final GoBildaPinpointDriver odometry;

  private final SwerveDriveKinematics kinematics;
  private final double drivebaseRadius;

  private final Module[] modules = new Module[4];

  public Swerve(OpMode opMode) {
    odometry = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "ODO");
    odometry.setOffsets(0, 0);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);

    double trackLengthMeters = 1;
    double trackWidthMeters = 1;
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(trackLengthMeters / 2, -trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, trackWidthMeters / 2),
            new Translation2d(-trackLengthMeters / 2, -trackWidthMeters / 2));
    drivebaseRadius = Math.hypot(trackLengthMeters / 2, trackWidthMeters / 2);

    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(opMode, i);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    var setpoint = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoint, Module.maxSpeedMetersPerSec);

    for (int i = 0; i < 4; i++) {
      modules[i].run(setpoint[i]);
    }
  }

  public void teleopDrive(double xInput, double yInput, double yawInput) {
    var translationalMagnitude = Math.hypot(xInput, yInput);
    if (translationalMagnitude > 1) {
      xInput /= translationalMagnitude;
      yInput /= translationalMagnitude;
    }

    drive(
        new ChassisSpeeds(
            xInput * Module.maxSpeedMetersPerSec,
            yInput * Module.maxSpeedMetersPerSec,
            yawInput * (Module.maxSpeedMetersPerSec / drivebaseRadius)));
  }

  public Pose2d getPose() {
    return new Pose2d(
        odometry.getPosX() / 1000.0,
        odometry.getPosY() / 1000.0,
        new Rotation2d(odometry.getHeading()));
  }

  public void periodic() {
    odometry.update();
  }

  private static final class Module {
    private static final double conversionFactor;
    static final double maxSpeedMetersPerSec;

    static {
      double countsPerRevolution = 537.7;
      double gearRatio = 1.7;
      double wheelCircumferenceMeters = (96.0 / 1000.0) * Math.PI;
      double maxMotorVelocity = 3.12;

      conversionFactor = countsPerRevolution * gearRatio / wheelCircumferenceMeters;
      maxSpeedMetersPerSec = ((maxMotorVelocity / gearRatio)) * wheelCircumferenceMeters;
    }

    final DcMotorEx driveMotor;
    final Servo steerServo;
    //    final CRServo steerServo;
    final AnalogInput steerEncoder;

    final PIDController drivePID;
    final SimpleMotorFeedforward driveFeedforward;

    Module(OpMode opMode, int id) {
      String pos;
      switch (id) {
        case 0 -> pos = "FL";
        case 1 -> pos = "FR";
        case 2 -> pos = "BL";
        case 3 -> pos = "BR";
        default -> throw new IllegalArgumentException("Module ID is out of range 0-3!");
      }

      driveMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get(pos + "Motor");
      steerServo = opMode.hardwareMap.servo.get(pos + "Servo");
      // steerServo = opMode.hardwareMap.crservo.get(pos + "Servo");
      steerEncoder = opMode.hardwareMap.analogInput.get(pos + "Encoder");

      drivePID = new PIDController(2 / maxSpeedMetersPerSec, 0, 0);
      driveFeedforward = new SimpleMotorFeedforward(0, 1 / maxSpeedMetersPerSec);
    }

    void run(SwerveModuleState state) {
      var servoPos = getServoPos();
      state.optimize(servoPos);
      state.cosineScale(servoPos);

      driveMotor.setPower(
          drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond)
              + driveFeedforward.calculate(state.speedMetersPerSecond));

      // TODO: Get continuous mode working
      steerServo.setPosition(
          MathUtil.inverseInterpolate(
              Units.degreesToRadians(-355.0 / 2),
              Units.degreesToRadians(355.0 / 2),
              MathUtil.angleModulus(state.angle.getRadians())));
    }

    public double getDrivePosition() {
      return driveMotor.getCurrentPosition() / conversionFactor;
    }

    private double lastPos;
    private double lastTime = -1;

    // We calculate motor velocity ourselves because REV sucks and only calculates velocity at 20
    // hz.
    public double getDriveVelocity() {
      if (lastTime == -1) {
        lastPos = getDrivePosition();
        lastTime = Utils.getTimeSeconds();
        return 0;
      }
      var currentPos = getDrivePosition();
      var currentTime = Utils.getTimeSeconds();
      var velocity = (currentPos - lastPos) / (currentTime - lastTime);
      lastPos = currentPos;
      lastTime = currentTime;
      return velocity;
    }

    private Rotation2d getServoPos() {
      return Rotation2d.fromDegrees(
          MathUtil.interpolate(
              0,
              360,
              MathUtil.inverseInterpolate(
                  0, steerEncoder.getMaxVoltage(), steerEncoder.getVoltage())));
    }

    private void runServoVel(double velRad) {
      double maxSpeedSecondsPer60Degrees = .115;
      double maxSpeedRadPerSec = (2 * Math.PI) / (maxSpeedSecondsPer60Degrees * 6);

      // steerServo.setPower(
      //     MathUtil.inverseInterpolate(-maxSpeedRadPerSec, maxSpeedRadPerSec, velRad) * 2 - 1);
    }
  }
}
