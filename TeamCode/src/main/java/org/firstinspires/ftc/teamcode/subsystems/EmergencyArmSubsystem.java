package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.Objects;

@Config
public class EmergencyArmSubsystem extends SubsystemBase {

    public DcMotorEx armMotorLower, armMotorHigher;
    public ServoEx pinchServo;
    public CRServo wristServo;

    public static final double THRESHOLD = .5f;

    public static double WRIST_SPEED = 1f;
    public static double LOWER_ARM_SPEED_PERCENT = 1;
    public static double HIGHER_ARM_SPEED_PERCENT = 1;
    public static double SET_POSITION_POWER = .55f;

    public static double LOWER_ARM_ANGLE_VELOCITY = 250;
    public static double HIGHER_ARM_ANGLE_VELOCITY = 250;

    public static double INITIAL_ANGLE_LOWER = 90-36.7;
    public static double INITIAL_ANGLE_HIGHER = 90-78.7;
    public static double LOWER_ARM_LENGTH = 10;
    public static double HIGHER_ARM_LENGTH = 7;
    public static double LOWER_GEAR_RATIO = 100;
    public static double HIGHER_GEAR_RATIO = 60;
    public static double TICKS_PER_REVOLUTION = 28;

    public static final boolean KEEP_PINCHER = false;

    private final static FTCDashboardPackets dbp = new FTCDashboardPackets("ArmSubsystem");
    Telemetry telemetry;

    public EmergencyArmSubsystem(HardwareMap map, Telemetry telemetry) throws Exception {
        this.telemetry = telemetry;
        armMotorLower = RobotHardwareInitializer.MotorComponent.LOWER_ARM.getEx(map);
        armMotorHigher = RobotHardwareInitializer.MotorComponent.HIGHER_ARM.getEx(map);
        if (KEEP_PINCHER) {
            pinchServo = RobotHardwareInitializer.ServoComponent.PINCHER.getEx(map);
            wristServo = RobotHardwareInitializer.CRServoComponent.WRIST.get(map);
        }

        armMotorLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorHigher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorHigher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorHigher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        haltLowerArm();
        haltHigherArm();
    }

    /**
     * @param power between [-1, 1]
     */
    public void setLowerArmPower(double power) {
        armMotorLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLower.setVelocity(power * LOWER_ARM_SPEED_PERCENT * LOWER_ARM_ANGLE_VELOCITY, AngleUnit.DEGREES);
        if (Math.abs(power) < KINEMATICS_DIVISOR_THRESHOLD) {
            armMotorLower.setVelocity(0, AngleUnit.DEGREES);
            haltLowerArm();
        }
    }

    /**
     * @param power between [-1, 1]
     */
    public void setHigherArmPower(double power) {
        armMotorHigher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorHigher.setVelocity(power * HIGHER_ARM_SPEED_PERCENT * HIGHER_ARM_ANGLE_VELOCITY, AngleUnit.DEGREES);
        if (Math.abs(power) < KINEMATICS_DIVISOR_THRESHOLD) {
            armMotorHigher.setPower(0);
            armMotorHigher.setVelocity(0, AngleUnit.DEGREES);
            haltHigherArm();
        }
    }

    public void haltLowerArm() {
        armMotorLower.setTargetPosition(armMotorLower.getCurrentPosition());
        armMotorLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLower.setPower(SET_POSITION_POWER);
    }

    public void haltHigherArm() {
        armMotorHigher.setTargetPosition(armMotorHigher.getCurrentPosition());
        armMotorHigher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorHigher.setPower(SET_POSITION_POWER);
    }

    final double KINEMATICS_DIVISOR_THRESHOLD = .2f;

    /**
     * Keeps the arm at a constant x position.
     * @param angularVelocityLowerUngeared the angle to move the lower arm
     */
    public void constantX(double angularVelocityLowerUngeared) {

        double thetaL = getAngleLower();
        double thetaH = getAngleHigher();

        double angularVelocityHigherUngeared = -LOWER_ARM_LENGTH * Math.sin(Math.toRadians(thetaL)) * angularVelocityLowerUngeared;
        double divisor = HIGHER_ARM_LENGTH * Math.sin(Math.toRadians(thetaH));

        if (divisor < KINEMATICS_DIVISOR_THRESHOLD) {
            armMotorHigher.setVelocity(0);
            armMotorLower.setVelocity(0);
            return;
        }

        angularVelocityHigherUngeared /= divisor;

        if (Math.abs(angularVelocityHigherUngeared) > 180) {
            // The angle is too big to keep x constant, and its too fast
            armMotorHigher.setVelocity(0);
            armMotorLower.setVelocity(0);
            return;
        }

        double angularVelocityHigherGeared = getAngularVelocityHigher(angularVelocityHigherUngeared);
        double angularVelocityLowerGeared = getAngularVelocityLower(angularVelocityLowerUngeared);
        armMotorHigher.setVelocity(angularVelocityHigherGeared, AngleUnit.DEGREES);
        armMotorLower.setVelocity(angularVelocityLowerGeared, AngleUnit.DEGREES);
    }

    public void constantY(double angularVelocityLowerUngeared) {

        double thetaL = getAngleLower();
        double thetaH = getAngleHigher();

        double angularVelocityHigherUngeared = -LOWER_ARM_LENGTH * Math.cos(Math.toRadians(thetaL)) * angularVelocityLowerUngeared;
        double divisor = HIGHER_ARM_LENGTH * Math.cos(Math.toRadians(thetaH));

        if (divisor < KINEMATICS_DIVISOR_THRESHOLD) {
            armMotorHigher.setVelocity(0);
            armMotorLower.setVelocity(0);
            return;
        }

        angularVelocityHigherUngeared /= divisor;

        if (Math.abs(angularVelocityHigherUngeared) > 180) {
            // The angle is too big to keep x constant, and its too fast
            armMotorHigher.setVelocity(0);
            armMotorLower.setVelocity(0);
            return;
        }

        double angularVelocityHigherGeared = getAngularVelocityHigher(angularVelocityHigherUngeared);
        double angularVelocityLowerGeared = getAngularVelocityLower(angularVelocityLowerUngeared);
        armMotorHigher.setVelocity(angularVelocityHigherGeared, AngleUnit.DEGREES);
        armMotorLower.setVelocity(angularVelocityLowerGeared, AngleUnit.DEGREES);
    }

    public enum PinchState {
        PINCHED(0), // left state
        OPEN(0.5f), // middle state
        ;

        public final double pinchPosition;
        PinchState(double pinchPosition) {
            this.pinchPosition = pinchPosition;
        }
    }

    public void setPinchState(PinchState state) {
        if (KEEP_PINCHER) {
            Objects.requireNonNull(state);
            pinchServo.setPosition(state.pinchPosition);
        }
    }

    public void closePincher() {
        setPinchState(PinchState.PINCHED);
    }

    public void openPincher() {
        setPinchState(PinchState.OPEN);
    }

    /**
     * @param power between [-1, 1]
     */
    public void setWristPower(double power) {
        if (KEEP_PINCHER) {
            wristServo.setPower(power * WRIST_SPEED);
            if (Math.abs(power) < KINEMATICS_DIVISOR_THRESHOLD) {
                wristServo.setPower(0);
            }
        }
    }

    /**
     * @return angle of the lower motor relative to the front horizontal in degrees
     */
    public double getAngleLower() {
        return encoderToAngleLower(armMotorLower.getCurrentPosition());
    }

    /**
     * @return angle of the higher motor relative to the front horizontal in degrees
     */
    public double getAngleHigher() {
        return encoderToAngleHigher(armMotorHigher.getCurrentPosition());
    }

    public double encoderToAngleLower(int encoderPosition) {
        double calculatedAngle = encoderPosition;
        calculatedAngle *= 360f;
        calculatedAngle /= LOWER_GEAR_RATIO * TICKS_PER_REVOLUTION;
        //calculatedAngle = (((float) encoderPosition) * 360f) / (LOWER_GEAR_RATIO * armMotorLower.getMotorType().getTicksPerRev());
        return calculatedAngle + INITIAL_ANGLE_LOWER;
    }

    public double encoderToAngleHigher(int encoderPosition) {
        double calculatedAngle = encoderPosition;
        calculatedAngle *= 360f;
        calculatedAngle /= HIGHER_GEAR_RATIO * TICKS_PER_REVOLUTION;
        return calculatedAngle + INITIAL_ANGLE_HIGHER;
    }

    public int angleToEncoderPositionLower(double angle) {
        angle *= TICKS_PER_REVOLUTION * LOWER_GEAR_RATIO;
        angle /= 360f;
        return (int) angle;
    }

    public int angleToEncoderPositionHigher(double angle) {
        angle *= TICKS_PER_REVOLUTION * HIGHER_GEAR_RATIO;
        angle /= 360f;
        return (int) angle;
    }

    /**
     * Scales the given angle by the gear ratio
     */
    public double getAngularVelocityLower(double angle) {
        return angle * LOWER_GEAR_RATIO;
    }

    /**
     * Scales the given angle by the gear ratio
     */
    public double getAngularVelocityHigher(double angle) {
        return angle * HIGHER_GEAR_RATIO;
    }

    public void moveToAngleLower(double angle) {
        armMotorLower.setTargetPosition(angleToEncoderPositionLower(angle));
        armMotorLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLower.setPower(SET_POSITION_POWER);
    }

    public void moveToAngleHigher(double angle) {
        armMotorHigher.setTargetPosition(angleToEncoderPositionHigher(angle));
        armMotorHigher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorHigher.setPower(SET_POSITION_POWER);
    }

    @Override
    public void periodic() {
        super.periodic();
        //dbp.info("Current position: "+armMotorLower.getCurrentPosition()+", "+armMotorLower.getTargetPosition()+"\n"+armMotorHigher.getCurrentPosition()+", "+armMotorHigher.getTargetPosition());
        dbp.info("Lower angle: "+getAngleLower()+"\nRight angle: "+getAngleHigher());
        dbp.send(true);
        telemetry.addData("Arm Angle", "Angle lower: "+getAngleLower()+"\nAngle higher"+getAngleHigher());
        telemetry.update();
    }
}
