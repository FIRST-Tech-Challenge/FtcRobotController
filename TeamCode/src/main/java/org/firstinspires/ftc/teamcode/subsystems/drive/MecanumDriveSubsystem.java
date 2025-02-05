package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;
import static org.firstinspires.ftc.teamcode.constants.Constants.DrivebaseConstants.*;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.OPTICAL_ODOMETRY_NAME;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.purepursuit.localization.Localizer;
import org.firstinspires.ftc.teamcode.purepursuit.localization.OpticalLocalizer;
import org.firstinspires.ftc.teamcode.utility.MotorUtility;

/**
 * <h1>Drive Subsystem</h1>
 * <br>
 * <p>
 *     Subsystem to encapsulate the drive motors. Contains the following hardware
 *     <ul>
 *         <li>Front Left Drive Motor</li>
 *         <li>Front Right Drive Motor</li>
 *         <li>Back Left Drive Motor</li>
 *         <li>Back Right Drive Motor</li>
 *         <li>Imu</li>
 *     </ul>
 * </p>
 */
//@TeleOp(name="NEW: Mecanum OpMode", group="Mecanum OpMode")
//@Disabled
public final class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotorImplEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private final IMU imu;

    private final Telemetry telemetry;
    private final DcMotorImplEx[] motors;
    public final Localizer localizer;

    /**
     * Constructs a new drive subsystems
     * @param opMode The opMode you are running ; To obtain the HardwareMap and Telemetry objects
     */
    public MecanumDriveSubsystem(@NonNull OpMode opMode) {
        telemetry = opMode.telemetry;

        HardwareMap hardwareMap = opMode.hardwareMap;

        localizer
                = new OpticalLocalizer(hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME));

        imu = hardwareMap.get(IMU.class, IMU_NAME);

        imu.initialize(IMU_PARAMETERS);
        imu.resetYaw();

        frontLeftMotor  = hardwareMap.get(DcMotorImplEx.class, FRONT_LEFT_DRIVE_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorImplEx.class, FRONT_RIGHT_DRIVE_MOTOR_NAME);
        backLeftMotor   = hardwareMap.get(DcMotorImplEx.class, BACK_LEFT_DRIVE_MOTOR_NAME);
        backRightMotor  = hardwareMap.get(DcMotorImplEx.class, BACK_RIGHT_DRIVE_MOTOR_NAME);

        motors = new DcMotorImplEx[]{
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

        MotorUtility.setDirections(REVERSE, frontLeftMotor, backLeftMotor);
        MotorUtility.setDirections(FORWARD, frontRightMotor, backRightMotor);

        MotorUtility.setMotorZeroPowerBehaviors(BRAKE, motors);
    }

    public void setFloat() {
        MotorUtility.setMotorZeroPowerBehaviors(FLOAT, motors);
    }

    /**
     * Drives the robot relative to itself
     * @param drive The drive value
     * @param strafe The strafe value
     * @param turn The turn value
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        if (Math.abs(drive)  < DRIVE_DEAD_ZONE)  drive  = 0.0;
        if (Math.abs(strafe) < STRAFE_DEAD_ZONE) strafe = 0.0;
        if (Math.abs(turn)   < TURN_DEAD_ZONE)   turn   = 0.0;

        double thetaRadians = StrictMath.atan2(drive, strafe);
        double power        = StrictMath.hypot(strafe, drive);

        double sinTheta = Math.sin(thetaRadians - Math.PI / 4.0);
        double cosTheta = Math.cos(thetaRadians - Math.PI / 4.0);

        double max = Math.max(Math.abs(cosTheta), Math.abs(sinTheta));

        double frontLeftPower  = power * cosTheta / max + turn;
        double frontRightPower = power * sinTheta / max - turn;
        double backLeftPower   = power * sinTheta / max + turn;
        double backRightPower  = power * cosTheta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Drives the robot relative to the field using the built-in imu
     * @param drive The value to move forward
     * @param strafe The value to strafe
     * @param turn The value to turn
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        if (Math.abs(drive)  < DRIVE_DEAD_ZONE)  drive  = 0.0;
        if (Math.abs(strafe) < STRAFE_DEAD_ZONE) strafe = 0.0;
        if (Math.abs(turn)   < TURN_DEAD_ZONE)   turn   = 0.0;

        double thetaRadians = StrictMath.atan2(drive, strafe);
        double power        = StrictMath.hypot(strafe, drive);

        thetaRadians -= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double sinTheta = Math.sin(Math.toRadians(thetaRadians));
        double cosTheta = Math.cos(Math.toRadians(thetaRadians));

        double max = Math.max(Math.abs(cosTheta), Math.abs(sinTheta));

        double frontLeftPower  = power * cosTheta / max + turn;
        double frontRightPower = power * sinTheta / max - turn;
        double backLeftPower   = power * sinTheta / max - turn;
        double backRightPower  = power * cosTheta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if ((power + turnMagnitude) > 1.0) {
            frontLeftPower  /= power + turnMagnitude;
            frontRightPower /= power + turnMagnitude;
            backLeftPower   /= power + turnMagnitude;
            backRightPower  /= power + turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Displays debug information about the drive base
     */
    public void debug() {
        telemetry.addLine("----- Drive Debug -----");
        telemetry.addData("Front Left Direction", frontLeftMotor.getDirection());
        telemetry.addData("Front Right Direction", frontRightMotor.getDirection());
        telemetry.addData("Back Left Direction", backLeftMotor.getDirection());
        telemetry.addData("Back Right Direction", backRightMotor.getDirection());
        telemetry.addData("Front Left Current", "%.3f", frontLeftMotor.getCurrent(AMPS));
        telemetry.addData("Front Right Current", "%.3f", frontRightMotor.getCurrent(AMPS));
        telemetry.addData("Back Left Current", "%.3f", backLeftMotor.getCurrent(AMPS));
        telemetry.addData("Back Right Current", "%.3f", backRightMotor.getCurrent(AMPS));
        telemetry.addData("Front Left Power", "%.3f", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", "%.3f", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", "%.3f",  backLeftMotor.getPower());
        telemetry.addData("Back Right Power", "%.3f", backRightMotor.getPower());
    }
}