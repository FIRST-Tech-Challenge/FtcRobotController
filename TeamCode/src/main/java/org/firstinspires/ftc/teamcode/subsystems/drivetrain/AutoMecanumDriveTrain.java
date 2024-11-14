package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * four wheel mecanum drive train for auto op mode, that hold references to odometers
 */
public class AutoMecanumDriveTrain extends FourWheelMecanumDrive {

    private static final double METER_PER_SEC_TO_POWER = 400;
    private final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    GoBildaPinpointDriver odo;

    ElapsedTime stopWatch = new ElapsedTime();

    long waitTimeNano = 300 * 1000;

    LimeLight limeLight;

    private MecanumDriveKinematics driveKinematics;

    public AutoMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, LimeLight limeLight) {
        super(hardwareMap, gamepad, telemetry, feedback, true);

        this.limeLight = limeLight;

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(Units.inchesToMMs(-6.5), Units.inchesToMMs(5.5));

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        Translation2d frontLeftWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d frontRightWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d rearLeftWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);
        Translation2d rearRightWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);

        driveKinematics = new MecanumDriveKinematics(
                frontLeftWheelMeters, frontRightWheelMeters,
                rearLeftWheelMeters, rearRightWheelMeters);


    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        super.createAndInitHardwares(hardwareMap);


    }

    @Override
    public void periodic() {
        super.periodic();

        odo.update();

    }

    public void DriveToTarget(double targetX, double targetY) {
        double minPower = 0.3;
        double distanceTolerance = 5;

        while (true) {
            odo.update();

            if(((Math.abs(targetX - odo.getPosX()) < distanceTolerance) && (Math.abs(targetY - odo.getPosY())) < distanceTolerance)) {
                // Give a 100ms to identify overshoot
                long endTime = System.nanoTime() + waitTimeNano;

                while(System.nanoTime() < endTime) {
                }

                if(((Math.abs(targetX - odo.getPosX()) < distanceTolerance) && (Math.abs(targetY - odo.getPosY())) < distanceTolerance)) {
                    break;
                }
            }

            // Battery reading of 13.49 required a Kp of 0.015
            double x = 0.0015*(targetX - odo.getPosX());
            double y = -0.0015*(targetY - odo.getPosY());

            double botHeading = odo.getHeading();

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            if(Math.abs(frontLeftPower) < minPower) {
                frontLeftPower = minPower * Math.signum(frontLeftPower);
            }

            if(Math.abs(frontRightPower) < minPower) {
                frontRightPower = minPower * Math.signum(frontRightPower);
            }

            if(Math.abs(backLeftPower) < minPower) {
                backLeftPower = minPower * Math.signum(backLeftPower);
            }

            if(Math.abs(backRightPower) < minPower) {
                backRightPower = minPower * Math.signum(backRightPower);
            }

            fL.motor.setPower(frontLeftPower);
            bL.motor.setPower(backLeftPower);
            fR.motor.setPower(frontRightPower);
            bR.motor.setPower(backRightPower);
        }

        Stop();

        telemetry.update();
    }

    private void Stop() {
        fL.motor.setPower(0);
        bL.motor.setPower(0);
        fR.motor.setPower(0);
        bR.motor.setPower(0);
    }


    public void TurnAngle(double targetAngleInDegrees) {
        double targetAngleInRadians = Math.toRadians(targetAngleInDegrees);
        double minError = Math.toRadians(1.5);

        double driveMotorsPower;
        double minPower = 0.2;

        odo.update();


        double error = targetAngleInRadians - odo.getHeading();

        while (true) {

            if(Math.abs(error) < minError) {
                // Give a 100ms to identify overshoot
                long endTime = System.nanoTime() + waitTimeNano;

                while(System.nanoTime() < endTime) {
                }

                if(Math.abs(error) < minError) {
                    break;
                }
            }

            driveMotorsPower = -.4 * error;

            if(Math.abs(driveMotorsPower) < minPower) {
                driveMotorsPower = minPower * Math.signum(driveMotorsPower);
            }

            // Positive power causes left turn
            fL.motor.setPower(-driveMotorsPower);
            bL.motor.setPower(-driveMotorsPower);
            fR.motor.setPower(driveMotorsPower);
            bR.motor.setPower(driveMotorsPower);

            odo.update();
            error = targetAngleInRadians - odo.getHeading();

        }

        fL.motor.setPower(0);
        bL.motor.setPower(0);
        fR.motor.setPower(0);
        bR.motor.setPower(0);
    }

    public boolean AlignTx() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;


        if(result != null) {
            double tx = result.getTx();
            telemetry.addLine("AlignTx");

            telemetry.addData("tx", tx);

            while(Math.abs(tx) > 1.0) {

                double power = tx * -0.01;
                if (Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                telemetry.addData("turn power", power);

                drive.driveRobotCentric(0, power, 0);

                result = limeLight.GetResult();

                if (result == null) {
                    break;
                }

                tx = result.getTx();

                telemetry.update();
            }
        }
        else {
            telemetry.addLine("No sample found");
            return false;
        }

        telemetry.update();

        drive.stop();
        return true;
    }

    public void AlignTy() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.2;

        if(result != null) {
            double ty = result.getTy() - 3.0;

            while(Math.abs(ty) > 1) {

                double power = ty * -0.01;
                if(Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                drive.driveRobotCentric(0, 0, -1 * power);

                result = limeLight.GetResult();

                if(result == null) {
                    telemetry.addLine("cannot find sample");
                    break;
                }

                ty = result.getTy();
            }

            drive.stop();
        }
    }

    public double GetPosX() {
        odo.update();

        return odo.getPosX();
    }

    public double GetPosY() {
        odo.update();

        return odo.getPosY();
    }

    public void Forward(double distanceInMm) {
        double min = 0.2;

        odo.update();
        double x1 = odo.getPosX();
        double y1 = odo.getPosY();

        while(true) {

                odo.update();
                double x2 = odo.getPosX();
                double y2 = odo.getPosY();

                double currentDistance = Math.sqrt((x2 - x1) * (x2 - x1)  + (y2 - y1) * (y2 - y1) );

                double error = distanceInMm - currentDistance;

                if(Math.abs(error) < 5) {
                    break;
                }


                double power = error * -0.02;
                if(Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                drive.driveRobotCentric(0, 0, -1 * power);
         }

            drive.stop();
    }


    public void TurnRelative(double turnInDegrees) {
        double min = 0.2;

        double turnInRadians = Math.toRadians(turnInDegrees);

        odo.update();
        double start = odo.getHeading();

        while(true) {

            odo.update();
            double current = odo.getHeading();

            double error = turnInRadians - current;

            if(Math.abs(error) < Math.toRadians(1)) {
                break;
            }

            double power = error * -0.02;
            if(Math.abs(power) < min) {
                power = min * Math.signum(power);
            }

            drive.driveRobotCentric(0, -power, 0);
        }

        drive.stop();
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }

    public void setWheelsPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        fL.motor.setPower(frontLeftPower);
        fR.motor.setPower(frontRightPower);
        bL.motor.setPower(backLeftPower);
        bR.motor.setPower(backRightPower);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void resetOdo() {
        odo.resetPosAndIMU();
    }

    public MecanumDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    private static Pose2d toPose2d(GoBildaPinpointDriver.Pose2D pose2D) {
        return new Pose2d(pose2D.getX(DistanceUnit.METER), pose2D.getY(DistanceUnit.METER),
                new Rotation2d(pose2D.getHeading(AngleUnit.RADIANS)));
    }

    public Pose2d getCurrentPose() {
        return toPose2d(odo.getPosition());
    }

    private double meterPerSecondToPower(double meterPerSec) {
        return meterPerSec * METER_PER_SEC_TO_POWER;
    }

    public void driveWithWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
        // Convert wheel speeds to motor powers
        double frontLeftPower = meterPerSecondToPower(wheelSpeeds.frontLeftMetersPerSecond);
        double frontRightPower = meterPerSecondToPower(wheelSpeeds.frontRightMetersPerSecond);
        double backLeftPower = meterPerSecondToPower(wheelSpeeds.rearLeftMetersPerSecond);
        double backRightPower = meterPerSecondToPower(wheelSpeeds.rearRightMetersPerSecond);

        // Normalize wheel speeds if any power is greater than 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        fL.set(frontLeftPower);
        fR.set(frontRightPower);
        bL.set(backLeftPower);
        bR.set(backRightPower);

        // Log data
        telemetry.addData("FL Speed (m/s)", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("FR Speed (m/s)", wheelSpeeds.frontRightMetersPerSecond);
        telemetry.addData("BL Speed (m/s)", wheelSpeeds.rearLeftMetersPerSecond);
        telemetry.addData("BR Speed (m/s)", wheelSpeeds.rearRightMetersPerSecond);
        telemetry.addData("FL Power", frontLeftPower);
        telemetry.addData("FR Power", frontRightPower);
        telemetry.addData("BL Power", backLeftPower);
        telemetry.addData("BR Power", backRightPower);
        telemetry.update();
    }
}
