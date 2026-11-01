package org.firstinspires.ftc.teamcode.scoring;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeController {

    private CRServo frontLeftFeeder;
    private CRServo frontRightFeeder;
    private CRServo windmillServo;
    private DcMotor frontIntakeMotor;

    /**
     * Initializes intake-side hardware used by scoring.
     *
     * Hardware names must match Robot Configuration:
     * - front_left_servo
     * - front_right_servo
     * - windmill_servo
     * - front_intake_motor
     */
    public void init(HardwareMap hardwareMap) {
        frontLeftFeeder = hardwareMap.get(CRServo.class, "front_left_servo");
        frontRightFeeder = hardwareMap.get(CRServo.class, "front_right_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");

        frontIntakeMotor = hardwareMap.get(DcMotor.class, "front_intake_motor");
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntakeMotor.setPower(0.0);

        stop();
    }

    /**
     * Applies one control step for intake + windmill.
     *
     * @param intakeMotorRequested true when driver wants front intake motor active
     * @param feederRequested true when driver wants side feeder servos active
     * @param windmillRequested true when shooter-feed assist should run windmill
     *
     * Behavior summary:
     * - Intake motor request powers intake motor.
     * - Feeder request powers both feeder servos.
     * - Windmill request spins windmill and enforces a minimum intake assist power.
     * - No request stops all intake-side motion.
     */
    public void applyIntakeAndWindmillControl(boolean intakeMotorRequested, boolean feederRequested, boolean windmillRequested) {
        double intakeMotorPower;

        if (feederRequested) {
            frontLeftFeeder.setPower(ScoringConfig.INTAKE_POWER);
            frontRightFeeder.setPower(-ScoringConfig.INTAKE_POWER);
        } else {
            frontLeftFeeder.setPower(0.0);
            frontRightFeeder.setPower(0.0);
        }

        if (intakeMotorRequested) {
            intakeMotorPower = ScoringConfig.INTAKE_POWER;
        } else {
            intakeMotorPower = 0.0;
        }

        if (windmillRequested) {
            windmillServo.setPower(-ScoringConfig.WINDMILL_POWER);
            intakeMotorPower = Math.max(intakeMotorPower, ScoringConfig.WINDMILL_INTAKE_ASSIST_POWER);
        } else {
            windmillServo.setPower(0.0);
        }

        frontIntakeMotor.setPower(intakeMotorPower);
    }

    public void stop() {
        if (frontLeftFeeder != null) {
            frontLeftFeeder.setPower(0.0);
        }
        if (frontRightFeeder != null) {
            frontRightFeeder.setPower(0.0);
        }
        if (windmillServo != null) {
            windmillServo.setPower(0.0);
        }
        if (frontIntakeMotor != null) {
            frontIntakeMotor.setPower(0.0);
        }
    }
}

