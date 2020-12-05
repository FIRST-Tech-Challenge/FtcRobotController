package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.control.AdaptivePurePursuitController;
import org.firstinspires.ftc.teamcode.lib.control.CurvePoint;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.team10515.Robot;
import org.firstinspires.ftc.teamcode.team10515.states.IState;

/**
 * This {@code Subsystem} represents a mecanum drivetrain, including drive code using the geometry
 * library, along with center-of-mass compensation, real-time trapezoidal motion profiling, and
 * static friction compensation.
 *
 * @see #drive(Pose2d, double)
 */
public class Drive implements ISubsystem {
    private static final double ACCELERATION_CAP = 2d; //motor power / s
    private static double lastPower;

    private RobotStateEstimator robotStateEstimator;

    private RevMotor backLeft;
    private RevMotor frontLeft;
    private RevMotor backRight;
    private RevMotor frontRight;

    private volatile Pose2d desiredPose;
    private volatile boolean updatedDesiredPose;

    private volatile double rmsFoundation;
    private volatile int rmsCount;

    public Drive(RobotStateEstimator robotStateEstimator, RevMotor backLeft, RevMotor frontLeft,
                 RevMotor backRight, RevMotor frontRight) {
        setRobotStateEstimator(robotStateEstimator);
        setBackLeft(backLeft);
        setFrontLeft(frontLeft);
        setBackRight(backRight);
        setFrontRight(frontRight);
        setLastPower(0d);
        setDesiredPose(new Pose2d());
        setUpdatedDesiredPose(false);
    }

    @Override
    public IState getStateMachine() {
        return null;
    }

    @Override
    public Enum getState() {
        return null;
    }

    @Override
    public void start() {

    }

    @Override
    public void update(double dt) {
        drive(dt);
    }

    @Override
    public void stop() {
        drive(Pose2d.identity(), 0d);
    }

    public Runnable autoMoveFooundation(Runnable toggleAutomation) {
        return () -> {
            ResidualVibrationReductionMotionProfilerGenerator motionProfile = ResidualVibrationReductionMotionProfilerGenerator.getFoundationMotionProfile();
            final double kP = 0.001d;
            final double kV = 1d / 40d;
            final double kA = 0d;
            final double initialPosition = getFrontLeft().getPosition();
            setRmsFoundation(0d);
            setRmsCount(0);
            motionProfile.start();
            while(!motionProfile.isDone()) {
                double error = motionProfile.getPosition() - (getFrontLeft().getPosition() - initialPosition);
                setRmsFoundation((getRmsFoundation() * getRmsCount() + error * error) / (getRmsCount() + 1));
                setRmsCount(getRmsCount() + 1);
                double output = kP * error + kV * motionProfile.getVelocity() + kA * motionProfile.getAcceleration();
                Robot.setDrivetrainPower(new Pose2d(new Translation2d(output, 0d), new Rotation2d(0d, false)));
                try {
                    Thread.sleep(40);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            setRmsFoundation(Math.sqrt(getRmsFoundation()));
            toggleAutomation.run();
        };
    }

    public Runnable driveToLocation(Runnable toggleAutomation, Pose2d desiredLocation, Gamepad gamepad1, Gamepad gamepad2) {
        setDesiredPose(desiredLocation);
        return driveToLocation(toggleAutomation, gamepad1, gamepad2);
    }

    public Runnable driveToLocation(Runnable toggleAutomation, Gamepad gamepad1, Gamepad gamepad2) {
        return () -> {
            Pose2d initialRobotLocation = getRobotStateEstimator().getPose();
            AdaptivePurePursuitController controller = new AdaptivePurePursuitController(
                    new CurvePoint(getDesiredPose().transformBy(initialRobotLocation), 10f)
            );

            while(!controller.hasReachedGoal() || Math.abs(gamepad1.left_stick_x) > 0.05 ||
                    Math.abs(gamepad1.left_stick_y) > 0.05 || Math.abs(gamepad1.right_stick_x) > 0.05 ||
                    Math.abs(gamepad1.right_stick_y) > 0.05 || Math.abs(gamepad2.left_stick_x) > 0.05 ||
                    Math.abs(gamepad2.left_stick_y) > 0.05 || Math.abs(gamepad2.right_stick_x) > 0.05 ||
                    Math.abs(gamepad2.right_stick_y) > 0.05) {
                if(hasUpdatedDesiredPose()) {
                    controller.resetPath(new CurvePoint(getDesiredPose().transformBy(getRobotStateEstimator().getPose()), 10f));
                    setUpdatedDesiredPose(false);
                }

                Robot.setDrivetrainPower(controller.follow(getRobotStateEstimator().getPose().transformBy(
                        initialRobotLocation.inverse()), getRobotStateEstimator().getVelocityPose().getTranslation().norm()));
                try {
                    Thread.sleep(40);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            toggleAutomation.run();
        };
    }

    public void drive(final double dt) {
        drive(Robot.getDrivetrainPower(), dt);
    }

    //TODO: Check signs
    public void drive(double backLeftPower, double frontLeftPower, double backRightPower, double frontRightPower) {
        double backLeftPowerMagnitude   = Math.abs(backLeftPower);
        double frontLeftPowerMagnitude  = Math.abs(frontLeftPower);
        double backRightPowerMagnitude  = Math.abs(backRightPower);
        double frontRightPowerMagnitude = Math.abs(frontRightPower);

        double highestPower = backLeftPowerMagnitude;
        if(frontLeftPowerMagnitude > highestPower) {
            highestPower = frontLeftPowerMagnitude;
        }

        if(backRightPowerMagnitude > highestPower) {
            highestPower = backRightPowerMagnitude;
        }

        if(frontRightPowerMagnitude > highestPower) {
            highestPower = frontRightPowerMagnitude;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        backLeftPower *= normalizationFactor;
        frontLeftPower *= normalizationFactor;
        backRightPower *= -normalizationFactor;
        frontRightPower *= -normalizationFactor;

        getBackLeft().setPower(backLeftPower);
        getFrontLeft().setPower(frontLeftPower);
        getBackRight().setPower(backRightPower);
        getFrontRight().setPower(frontRightPower);
    }

    public void drive(final Pose2d power, final double dt) {
        double robotPower = power.getTranslation().norm() + Math.abs(power.getRotation().getRadians());
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        double acceleration = (robotPower - getLastPower()) / dt;
        if(acceleration > getAccelerationCap()) {
            robotPower = getAccelerationCap() * dt + getLastPower();
        } if(acceleration < -getAccelerationCap()) {
            robotPower = -getAccelerationCap() * dt + getLastPower();
        }

        setLastPower(robotPower);

        double v1 = x - y + (Robot.getL1() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v2 = x + y - (Robot.getL1() + Robot.getD2()) * turn / Robot.getRobotLength();
        double v3 = x + y + (Robot.getL2() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v4 = x - y - (Robot.getL2() + Robot.getD2()) * turn / Robot.getRobotLength();

        v1 *= robotPower;
        v2 *= robotPower;
        v3 *= robotPower;
        v4 *= robotPower;

        //v1 += 0.15d * Math.signum(v1);
        //v2 += 0.15d * Math.signum(v2);
        //v3 += 0.15d * Math.signum(v3);
        //v4 += 0.15d * Math.signum(v4);

        double highestPower = v1;
        if(v2 > highestPower) {
            highestPower = v2;
        }

        if(v3 > highestPower) {
            highestPower = v3;
        }

        if(v4 > highestPower) {
            highestPower = v4;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        v1 *= normalizationFactor;
        v2 *= normalizationFactor;
        v3 *= normalizationFactor;
        v4 *= normalizationFactor;

        getBackLeft().setPower(v3);
        getFrontLeft().setPower(v1);
        getBackRight().setPower(v4);
        getFrontRight().setPower(v2);
    }

    /*public void drive(final DoubleMatrix input) {
        drive(input.get(0), input.get(1), input.get(2), input.get(3));
    }*/

    public void drive(final Pose2d power) {
        double robotPower = power.getTranslation().norm() + Math.abs(power.getRotation().getRadians());
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        setLastPower(robotPower);

        double v1 = -x + y - (Robot.getL1() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v2 = -x - y + (Robot.getL1() + Robot.getD2()) * turn / Robot.getRobotLength();
        double v3 = -x - y - (Robot.getL2() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v4 = -x + y + (Robot.getL2() + Robot.getD2()) * turn / Robot.getRobotLength();

        v1 += 0.15d * Math.signum(v1);
        v2 += 0.15d * Math.signum(v2);
        v3 += 0.15d * Math.signum(v3);
        v4 += 0.15d * Math.signum(v4);

        double highestPower = v1;
        if(v2 > highestPower) {
            highestPower = v2;
        }

        if(v3 > highestPower) {
            highestPower = v3;
        }

        if(v4 > highestPower) {
            highestPower = v4;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        v1 *= normalizationFactor * robotPower;
        v2 *= normalizationFactor * robotPower;
        v3 *= normalizationFactor * robotPower;
        v4 *= normalizationFactor * robotPower;

        getBackLeft().setPower(v3);
        getFrontLeft().setPower(v1);
        getBackRight().setPower(v4);
        getFrontRight().setPower(v2);
    }

    public void drive(SimpleMatrix input) {
        drive(input.get(0), input.get(1), input.get(2), input.get(3));
    }

    /*public DoubleMatrix getLastInput() {
        return new DoubleMatrix(4, 1, new double[] {
                getBackLeft().getLastPower(),
                getFrontLeft().getLastPower(),
                getBackRight().getLastPower(),
                getFrontRight().getLastPower()
        });
    }*/

    public SimpleMatrix getLastInput() {
        return new SimpleMatrix(4, 1, false, new double[] {
                getBackLeft().getLastPower(),
                getFrontLeft().getLastPower(),
                getBackRight().getLastPower(),
                getFrontRight().getLastPower()
        });
    }

    public RevMotor getBackLeft() {
        return backLeft;
    }

    public void setBackLeft(RevMotor backLeft) {
        this.backLeft = backLeft;
    }

    public RevMotor getFrontLeft() {
        return frontLeft;
    }

    public void setFrontLeft(RevMotor frontLeft) {
        this.frontLeft = frontLeft;
    }

    public RevMotor getBackRight() {
        return backRight;
    }

    public void setBackRight(RevMotor backRight) {
        this.backRight = backRight;
    }

    public RevMotor getFrontRight() {
        return frontRight;
    }

    public void setFrontRight(RevMotor frontRight) {
        this.frontRight = frontRight;
    }

    public static double getLastPower() {
        return lastPower;
    }

    public static void setLastPower(double lastPower) {
        Drive.lastPower = lastPower;
    }

    public static double getAccelerationCap() {
        return ACCELERATION_CAP;
    }

    public double getRmsFoundation() {
        return rmsFoundation;
    }

    public void setRmsFoundation(double rmsFoundation) {
        this.rmsFoundation = rmsFoundation;
    }

    public int getRmsCount() {
        return rmsCount;
    }

    public void setRmsCount(int rmsCount) {
        this.rmsCount = rmsCount;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Pose2d getDesiredPose() {
        return desiredPose;
    }

    public void setDesiredPose(Pose2d desiredPose) {
        setUpdatedDesiredPose(true);
        this.desiredPose = desiredPose;
    }

    public boolean hasUpdatedDesiredPose() {
        return updatedDesiredPose;
    }

    public void setUpdatedDesiredPose(boolean updatedDesiredPose) {
        this.updatedDesiredPose = updatedDesiredPose;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }
}
