package Utils.Chassis;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Utils.AdvancedPidController;
import org.firstinspires.ftc.teamcode.Utils.DepthCamera;

@Config
public class ChassisDriver {
    public static double MIN_POWER_OFFSET = 0.14;
    public static double STRAFE_POWER_MULTIPLIER = 2;

    public static double forwardP = 0.00017;
    public static double forwardI = 0;
    public static double forwardD = 0;
    public static double forwardT = 100;
    public static double forwardMaxIOutput = 2;
    public static double forwardPowerLimit = 0.3;
    AdvancedPidController forwardPid = new AdvancedPidController(forwardP, forwardI, forwardD, forwardMaxIOutput, "Forward Pid");

    public static double leftP = 0.00015;
    public static double leftI = 0;
    public static double leftD = 0;
    public static double leftT = 9;
    public static double leftMaxIOutput = 2;
    public static double leftPowerLimit = 1;
    AdvancedPidController leftPid = new AdvancedPidController(leftP, leftI, leftD, leftMaxIOutput, "Left Pid");

    public static double rotationP = 8;
    public static double rotationI = 0;
    public static double rotationD = 0;
    public static double rotationT = 0.04;
    public static double rotationMaxIOutput = 2;
    public static double rotationPowerLimit = 1.5;
    AdvancedPidController rotationPid = new AdvancedPidController(rotationP, rotationI, rotationD, rotationMaxIOutput, "Rotation Pid");

    public static double FAST_SPEED_MULTIPLIER = 2;
    public static double FAST_TURN_MULTIPLIER = 4;
    public static double SLOW_SPEED_MULTIPLIER = 0.8;
    public static double SLOW_TURN_MULTIPLIER = 1.5;

    private final static Translation2d lfLocation = new Translation2d(0.095, 0.096);
    private final static Translation2d rfLocation = new Translation2d(0.095, -0.096);
    private final static Translation2d lbLocation = new Translation2d(-0.095, 0.096);
    private final static Translation2d rbLocation = new Translation2d(-0.095, -0.096);
    private final static MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(lfLocation, rfLocation, lbLocation, rbLocation);

    static DcMotorEx lf, rf, lb, rb;
    IMU imu;

    DepthCamera frontDepthCamera;
    DepthCamera backDepthCamera;
    FieldOrientedDriver fieldOrientedDriver;

    public ChassisDriver(IMU imu) {
        this.imu = imu;

        resetPIDs();

        fieldOrientedDriver = new FieldOrientedDriver(imu);
    }

    public ChassisDriver(DepthCamera backDepthCamera, IMU imu) {
        this.backDepthCamera = backDepthCamera;
        this.imu = imu;

        resetPIDs();

        fieldOrientedDriver = new FieldOrientedDriver(imu);
    }

    public ChassisDriver(DepthCamera frontDepthCamera, DepthCamera backDepthCamera, IMU imu) {
        this.frontDepthCamera = frontDepthCamera;
        this.backDepthCamera = backDepthCamera;
        this.imu = imu;

        resetPIDs();

        fieldOrientedDriver = new FieldOrientedDriver(imu);
    }

    public static void initializeMotors(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        ChassisDriver.lf = lf;
        ChassisDriver.rf = rf;
        ChassisDriver.lb = lb;
        ChassisDriver.rb = rb;

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void resetWheelEncoders(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        DcMotor.RunMode lfRM = lf.getMode();
        DcMotor.RunMode rfRM = rf.getMode();
        DcMotor.RunMode lbRM = lb.getMode();
        DcMotor.RunMode rbRM = rb.getMode();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(lfRM);
        rf.setMode(rfRM);
        lb.setMode(lbRM);
        rb.setMode(rbRM);
    }

    public static void fieldOrientedDrive(double forwardPower, double leftPower, double rotationPower, Rotation2d heading) {
    }

    public static void resetWheelEncoders() {
    }

    public void setNormalizedDrive(Pose2d drivePower) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                drivePower.getX(),
                drivePower.getY(),
                drivePower.getHeading()
        );

        MecanumDriveWheelSpeeds mecanumWheelSpeeds = MECANUM_KINEMATICS.toWheelSpeeds(speeds);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = mecanumWheelSpeeds.frontLeftMetersPerSecond;
        wheelSpeeds[1] = mecanumWheelSpeeds.frontRightMetersPerSecond;
        wheelSpeeds[2] = mecanumWheelSpeeds.rearLeftMetersPerSecond;
        wheelSpeeds[3] = mecanumWheelSpeeds.rearRightMetersPerSecond;

        adjustedWheelSpeeds(wheelSpeeds);

        requestPowers(
                wheelSpeeds[0],
                wheelSpeeds[1],
                wheelSpeeds[2],
                wheelSpeeds[3]
        );
    }

    public static void rawDrive(double forwardValue, double leftValue, double rotationValue) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                forwardValue,
                leftValue,
                rotationValue
        );

        MecanumDriveWheelSpeeds wheelSpeeds = MECANUM_KINEMATICS.toWheelSpeeds(speeds);

        requestPowers(
                wheelSpeeds.frontLeftMetersPerSecond,
                wheelSpeeds.frontRightMetersPerSecond,
                wheelSpeeds.rearLeftMetersPerSecond,
                wheelSpeeds.rearRightMetersPerSecond
        );
    }

    public static void rawDrive(Pose2d drivePower) {
        rawDrive(drivePower.getX(), drivePower.getY(), drivePower.getHeading());
    }

    public void updateLineUpDrive(double targetHeading) {
        Pose2d backDepthError = backDepthCamera.getDepthError();
        double forwardError = backDepthError.getX();
        double leftError = backDepthError.getY();
        double rError = fieldOrientedDriver.getCalibratedHeading() - targetHeading;

        if (rError > Math.PI) {
            rError -= 2 * Math.PI;
        } else if (rError < -Math.PI) {
            rError += 2 * Math.PI;
        }

        Log.d("rError", String.valueOf(rError));

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower;
        if (Math.abs(leftError) < leftT) {
            leftPower = 0;
        } else {
            leftPower = leftPid.calculate(leftError);
        }

        rawDrive(forwardPower, leftPower, rPower);
    }

    public void updateLineUpDrive(int targetDistance, double targetHeading) {
        Pose2d backDepthError = backDepthCamera.getDepthErrorAdvanced(targetDistance);
        double forwardError = backDepthError.getX();
        double leftError = backDepthError.getY();
        double rError = fieldOrientedDriver.getCalibratedHeading() - targetHeading;

        if (rError > Math.PI) {
            rError -= 2 * Math.PI;
        } else if (rError < -Math.PI) {
            rError += 2 * Math.PI;
        }

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower;
        if (Math.abs(leftError) < leftT) {
            leftPower = 0;
        } else {
            leftPower = leftPid.calculate(leftError);
        }

        rawDrive(forwardPower, leftPower, rPower);
    }


    public Pose2d updatePickUpDrive() {
        Pose2d frontDepthError = frontDepthCamera.getDepthError();
        double forwardError = frontDepthError.getX();
        double rError = -frontDepthError.getHeading();

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower = 0;

        rawDrive(-forwardPower, leftPower, rPower);

        return frontDepthError;
    }

    public Pose2d updateDropOffDrive(int targetD, int predictionDistance) {
        Pose2d backDepthError = backDepthCamera.getDepthError(targetD);
        double forwardError = backDepthError.getX() + predictionDistance;
        double rError = -backDepthError.getHeading();

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower = 0;

        rawDrive(forwardPower, leftPower, rPower);

        return backDepthError;
    }

    public Pose2d updateDropOffDrive(int targetD) {
        Pose2d backDepthError = backDepthCamera.getDepthError(targetD);
        double forwardError = backDepthError.getX();
        double rError = -backDepthError.getHeading();

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower = 0;

        rawDrive(forwardPower, leftPower, rPower);

        return backDepthError;
    }

    public Pose2d updateDropOffDriveLowJunction(int targetD) {
        Pose2d frontDepthError = frontDepthCamera.getDepthError(targetD);
        double forwardError = -frontDepthError.getX();
        double rError = -frontDepthError.getHeading();

        double forwardPower;
        if (Math.abs(forwardError) < forwardT) {
            forwardPower = 0;
        } else {
            forwardPower = forwardPid.calculate(forwardError);
        }

        double rPower;
        if (Math.abs(rError) < rotationT) {
            rPower = 0;
        } else {
            rPower = rotationPid.calculate(rError);
        }

        double leftPower = 0;

        rawDrive(forwardPower, leftPower, rPower);

        return frontDepthError;
    }

    public void resetPIDs() {
        forwardPid.reset();
        forwardPid.setPID(forwardP, forwardI, forwardD);
        forwardPid.setMaxIOutput(forwardMaxIOutput);
        forwardPid.setOutputLimits(forwardPowerLimit);

        leftPid.reset();
        leftPid.setPID(leftP, leftI, leftD);
        leftPid.setMaxIOutput(leftMaxIOutput);
        leftPid.setOutputLimits(leftPowerLimit);

        rotationPid.reset();
        rotationPid.setPID(rotationP, rotationI, rotationD);
        rotationPid.setMaxIOutput(rotationMaxIOutput);
        rotationPid.setOutputLimits(rotationPowerLimit);
    }

    public void updateFieldOrientedDrive(double forwardValue, double leftValue, double rotationValue) {
        ChassisSpeeds speeds = fieldOrientedDriver.getChassisSpeeds(forwardValue, leftValue, rotationValue);

        MecanumDriveWheelSpeeds wheelSpeeds = MECANUM_KINEMATICS.toWheelSpeeds(speeds);

        requestPowers(
                wheelSpeeds.frontLeftMetersPerSecond,
                wheelSpeeds.frontRightMetersPerSecond,
                wheelSpeeds.rearLeftMetersPerSecond,
                wheelSpeeds.rearRightMetersPerSecond
        );
    }

    /**
     * FOD with normalized wheel powers.
     *
     * @param drivePower
     */
    public void setNormalizedFieldOrientedDrive(Pose2d drivePower) {
        ChassisSpeeds speeds = fieldOrientedDriver.getChassisSpeeds(
                drivePower.getX(),
                drivePower.getY(),
                drivePower.getHeading()
        );

        MecanumDriveWheelSpeeds mecanumWheelSpeeds = MECANUM_KINEMATICS.toWheelSpeeds(speeds);
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = mecanumWheelSpeeds.frontLeftMetersPerSecond;
        wheelSpeeds[1] = mecanumWheelSpeeds.frontRightMetersPerSecond;
        wheelSpeeds[2] = mecanumWheelSpeeds.rearLeftMetersPerSecond;
        wheelSpeeds[3] = mecanumWheelSpeeds.rearRightMetersPerSecond;
        adjustedWheelSpeeds(wheelSpeeds);

        requestPowers(
                wheelSpeeds[0],
                wheelSpeeds[1],
                wheelSpeeds[2],
                wheelSpeeds[3]
        );
    }

    public void resetFieldOrientedDriveForwardHeading() {
        fieldOrientedDriver.resetCalibratedForwardHeading();
    }

    public void setFODTargetHeadingRad(double a) {
        fieldOrientedDriver.setTargetHeadingRad(a);
    }

    public static void requestPowers(double lfPower, double rfPower, double lbPower, double rbPower) {
        lf.setPower(adjustMotorPower(lfPower));
        rf.setPower(adjustMotorPower(rfPower));
        lb.setPower(adjustMotorPower(lbPower));
        rb.setPower(adjustMotorPower(rbPower));
    }

    private static double adjustMotorPower(double p) {
        if (p == 0)
            return 0;
        else if (p > 0)
            return MIN_POWER_OFFSET + (1 - MIN_POWER_OFFSET) * p;
        else

            return -MIN_POWER_OFFSET + (1 - MIN_POWER_OFFSET) * p;
    }

    public double getCalibratedHeading() {
        return fieldOrientedDriver.getCalibratedHeading();
    }

    public void setCalibratedForwardHeading(double a) {
        fieldOrientedDriver.setCalibratedForwardHeading(a);
    }

    public void adjustedWheelSpeeds(double[] wheelSpeeds) {
        double maxAbsolute = wheelSpeeds[0];
        for (int i = 1; i < 4; i++) {
            maxAbsolute = Math.max(maxAbsolute, Math.abs(wheelSpeeds[i]));
        }
        if (maxAbsolute > 1.0) {
            for (int i = 0; i < 4; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxAbsolute;
            }
        }
    }
}