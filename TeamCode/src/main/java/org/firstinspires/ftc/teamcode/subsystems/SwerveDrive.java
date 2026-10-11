package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveKinematics;
import org.firstinspires.ftc.teamcode.teleop.SwerveTeleOpConfig;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

public class SwerveDrive {

    private Telemetry telemetry;

    private GoBildaPinpointDriver odo = null;

    private final myDcMotorEx mod1m1;
    private final myDcMotorEx mod1m2;
    private final myDcMotorEx mod2m1;
    private final myDcMotorEx mod2m2;
    private final AnalogInput mod1E;
    private final AnalogInput mod2E;

    // PIDs
    private final PIDcontroller mod1PID = new PIDcontroller(0, 0, 0, 0, 0);
    private final PIDcontroller mod2PID = new PIDcontroller(0, 0, 0, 0, 0);

    private final swerveKinematics kinematics = new swerveKinematics();

    private double imuOffset = 0;
    private boolean imuZeroed = false;

    // Heading correction (ported closely to user's previous implementation)
    private final ElapsedTime headingDt = new ElapsedTime();
    private final ElapsedTime headingReset = new ElapsedTime();
    private double headingTarget = 0;
    private double lastHeadingError = 0;
    private boolean headingTargetSet = false;
    private double HEADING_LOCK_DEADBAND = SwerveTeleOpConfig.HEADING_LOCK_DEADBAND;

    // Safety Variables
    private boolean initialized = false;
    private double lastGoodHeading = 180; // Failsafe for IMU singularity

    public SwerveDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        // --- Hardware Mapping ---
        mod1m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m1"));
        mod1m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod1m2"));
        mod2m1 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m1"));
        mod2m2 = new myDcMotorEx(hardwareMap.get(DcMotorEx.class, "mod2m2"));

        mod1E = hardwareMap.get(AnalogInput.class, "mod1E");
        mod2E = hardwareMap.get(AnalogInput.class, "mod2E");

        // --- Motor Defaults ---
        myDcMotorEx[] allMotors = {mod1m1, mod1m2, mod2m1, mod2m2};
        for (myDcMotorEx motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPowerThresholds(0.05, 0.0);
        }

        // --- Motor Reversals ---
        // Only bottom motors are reversed (Coaxial standard)
        mod1m2.setDirection(DcMotorSimple.Direction.FORWARD);
        mod2m2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveWithConfig(double strafe, double forward, double rot) {
        drive(
                strafe, forward, rot,
                SwerveTeleOpConfig.module1Adjust,
                SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust,
                SwerveTeleOpConfig.Kp,
                SwerveTeleOpConfig.Kd,
                SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf,
                SwerveTeleOpConfig.Kl,
                SwerveTeleOpConfig.FIELD_CENTRIC,
                SwerveTeleOpConfig.IMU_POLARITY,
                SwerveTeleOpConfig.ROBOT_RADIUS
        );
    }

    public void drive(double strafe, double forward, double rot,
                      double m1Offset, double m2Offset, double m3Offset,
                      double Kp, double Kd, double Ki, double Kf, double Kl,
                      boolean fieldCentric, double imuPolarity, double robotRadius) {
        double strafeIn = strafe;
        double forwardIn = forward;
        double rotIn = rot;

        HEADING_LOCK_DEADBAND = SwerveTeleOpConfig.HEADING_LOCK_DEADBAND;

        GoBildaPinpointDriver odo = this.odo;
        if (odo != null) {
            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
            // Auto-zero IMU once at startup using the same math as getHeading(), so heading reads 0 at startup.
            if (!imuZeroed) {
                double rawDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS));
                imuOffset = rawDeg * SwerveTeleOpConfig.IMU_POLARITY - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG;
                imuZeroed = true;
            }
        }

        if (Double.isNaN(forwardIn) || Double.isNaN(strafeIn) || Double.isNaN(rotIn)) {
            forwardIn = 0;
            strafeIn = 0;
            rotIn = 0;
        }

        // 1. Update PID Coefficients
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);

        // 2. Read Sensors (Volts -> Degrees)
        double mod1P = readEncoder(mod1E, m1Offset);
        double mod2P = readEncoder(mod2E, m2Offset);

        double heading = 0;
        boolean needsHeading = fieldCentric || (initialized && Math.abs(rotIn) < HEADING_LOCK_DEADBAND);
        if (needsHeading) {
            if (odo == null) {
                heading = lastGoodHeading;
                telemetry.addData("WARNING", "IMU UNAVAILABLE - USING FALLBACK");
            } else {
                double rawHeading = getHeading(imuPolarity);

                if (Double.isNaN(rawHeading)) {
                    heading = lastGoodHeading; // Use last known safe angle
                    telemetry.addData("WARNING", "IMU NAN DETECTED - USING FALLBACK");
                } else {
                    heading = rawHeading;
                    lastGoodHeading = heading; // Update history
                }
            }
        }

        double headingCorrection = 0;
        if (needsHeading && odo != null) {
            // First-time capture after init/reset so we don't drive toward 0 by default
            if (!headingTargetSet) {
                headingTarget = heading;
                headingTargetSet = true;
                headingReset.reset();
                headingDt.reset();
            }
            if (Math.abs(rotIn) > 0.05) {
                headingReset.reset();
            }
            if (headingReset.milliseconds() < 200) {
                headingTarget = heading;
                lastHeadingError = 0;
            } else {
                double headingError = AngleUnit.normalizeDegrees(headingTarget - heading);
                double dt = headingDt.seconds();
                if (dt <= 0) dt = 1e-3;

                headingCorrection = SwerveTeleOpConfig.HEADING_LOCK_KP * headingError +
                        SwerveTeleOpConfig.HEADING_LOCK_KD * (headingError - lastHeadingError) / dt;

                lastHeadingError = headingError;
            }
            headingDt.reset();
        }

        rotIn -= headingCorrection;
        rotIn = Range.clip(rotIn, -1.0, 1.0);

        // 5. Calculate Kinematics (Vectors)
        double[] output = kinematics.calculate(forwardIn, -strafeIn, -rotIn, heading, fieldCentric, robotRadius);

        double mod1power = output[0];
        double mod2power = output[1];

        // References
        double mod1reference = output[2];
        double mod2reference = output[3];

        // 6. Locking Logic
        if (forwardIn != 0 || strafeIn != 0 || rotIn != 0 || !initialized) {
            initialized = true;
        }

        // 7. Efficient Turn & PID
        // Angle Wrap
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);

        mod1reference = mathsOperations.angleWrap(mod1reference);
        mod2reference = mathsOperations.angleWrap(mod2reference);

        // Efficient Turn
        double[] m1Eff = mathsOperations.efficientTurn(mod1reference, mod1P, mod1power);
        double[] m2Eff = mathsOperations.efficientTurn(mod2reference, mod2P, mod2power);

        // PID Calculation
        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));

        // 8. Differential Mixing (PID + Drive Power)
        double[] m1Out = mathsOperations.diffyConvert(-m1PID, m1Eff[1]);
        double[] m2Out = mathsOperations.diffyConvert(-m2PID, m2Eff[1]);

        // 9. Output
        mod1m1.setPower(m1Out[0]);
        mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]);
        mod2m2.setPower(m2Out[1]);

        // 10. Telemetry
        telemetry.addData("Heading", heading);
        telemetry.addData("M1 Angle/Ref", "%.1f / %.1f", mod1P, m1Eff[0]);
        telemetry.addData("M2 Angle/Ref", "%.1f / %.1f", mod2P, m2Eff[0]);
    }

    private double readEncoder(AnalogInput enc, double offset) {
        // Parentheses here are critical! Do not remove.
        double raw = (enc.getVoltage() - 0.043) / 3.1 * 360.0;
        return AngleUnit.normalizeDegrees(raw - offset);
    }

    private double getHeading(double polarity) {
        double headingDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS));
        return AngleUnit.normalizeDegrees(
                headingDeg * polarity - imuOffset - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG
        );
    }

    public void resetIMU() {
        GoBildaPinpointDriver odo = this.odo;
        if (odo != null) {
            double rawDeg = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS));
            imuOffset = rawDeg * SwerveTeleOpConfig.IMU_POLARITY - SwerveTeleOpConfig.HEADING_FRAME_OFFSET_DEG;
        }
        headingTarget = Math.PI;
        lastHeadingError = 0;
        headingDt.reset();
        headingReset.reset();
        headingTargetSet = false;
        lastGoodHeading = 0;
        initialized = false; // re-arm lock on next driver input
        imuZeroed = true;
    }

    public void tuneModules(double targetAngle, double m1Offset, double m2Offset, double m3Offset,
                            double Kp, double Kd, double Ki, double Kf, double Kl) {
        // Update PIDs
        mod1PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);
        mod2PID.setPIDgains(Kp, Kd, Ki, Kf, Kl);

        // Read Current Angles
        double mod1P = readEncoder(mod1E, m1Offset);
        double mod2P = readEncoder(mod2E, m2Offset);

        // Wrap/Optimize
        mod1P = mathsOperations.angleWrap(mod1P);
        mod2P = mathsOperations.angleWrap(mod2P);

        double[] m1Eff = mathsOperations.efficientTurn(targetAngle, mod1P, 0.0);
        double[] m2Eff = mathsOperations.efficientTurn(targetAngle, mod2P, 0.0);

        double m1PID = mod1PID.pidOut(AngleUnit.normalizeDegrees(m1Eff[0] - mod1P));
        double m2PID = mod2PID.pidOut(AngleUnit.normalizeDegrees(m2Eff[0] - mod2P));

        // Apply ONLY rotation power (no drive)
        double[] m1Out = mathsOperations.diffyConvert(m1PID, 0.0);
        double[] m2Out = mathsOperations.diffyConvert(-m2PID, 0.0);

        mod1m1.setPower(m1Out[0]);
        mod1m2.setPower(m1Out[1]);
        mod2m1.setPower(m2Out[0]);
        mod2m2.setPower(m2Out[1]);

        telemetry.addData("Target", targetAngle);
        telemetry.addData("M1 Pos", mod1P);
        telemetry.addData("M2 Pos", mod2P);
    }

    public void updateTelemetry(Telemetry telem) {
        telemetry = telem;
    }

    public void updateOdo(GoBildaPinpointDriver odom) {
        odo = odom;
    }
}
