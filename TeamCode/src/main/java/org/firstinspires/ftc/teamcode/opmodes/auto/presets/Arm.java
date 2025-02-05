package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.ClawCode;

public class Arm {
    public final DcMotorEx armMotor;
    private static final double ARM_POWER = 0.5;

    private static final double RETRACT_ARM_POWER = -0.5;

    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 14.0;
    private static final double GROUND_DEGREES = 10.0;   // Default position (0 degrees)

    private static final double MIN_DEGREES_FOR_WRIST = 28.0;

    private static final double DEGREES_FOR_SPECIMEN_ON_WALL = 15.0;

    private static final double DEGREES_FOR_SPECIMEN_ON_FLOOR = 5.0;

    private static final double DEGREES_FOR_MOVING_HEIGHT = 30.0;

    private static final double ARM_POSITION_FOR_MOVING_HEIGHT = 600.0;

    private static final double ARM_POSITION_FOR_HIGH_RUNG = 1425.2;

    private static final double ARM_POSITION_FOR_INIT = 200;

    private static final double LOW_DEGREES = 12.0;     // Position to pick up from the ground (15 degrees)
    private static final double HIGH_DEGREES = 71.0;    // Position to place into low basket (45 degrees)
    private static final double MAX_DEGREES = 95.0;     // Position to place into an high basket (70 degrees)

    // Formula to calculate ticks per degree
    final double ARM_TICKS_PER_DEGREE =
//            19.2032086;
            145.1 // encoder ticks per rotation of the bare RS-555 motor
                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
                    * 1 / 360.0 * 2; // we want ticks per degree, not per rotation


    // Pre-calculated arm positions in encoder ticks based on degrees
    private final double INIT_POSITION_TICKS = INIT_DEGREES* ARM_TICKS_PER_DEGREE;
    private final double GROUND_POSITION_TICKS = GROUND_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double MIN_POS_FOR_WRIST_TICKS = MIN_DEGREES_FOR_WRIST * ARM_TICKS_PER_DEGREE;
    private final double LOW_POSITION_TICKS = LOW_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double HIGH_POSITION_TICKS = HIGH_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double MAX_POSITION_TICKS = MAX_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double DEGREES_FOR_MOVING_HEIGHT_TICKS = DEGREES_FOR_MOVING_HEIGHT * ARM_TICKS_PER_DEGREE;
    private final double DEGREES_FOR_SPECIMEN_ON_FLOOR_TICKS = DEGREES_FOR_SPECIMEN_ON_FLOOR * ARM_TICKS_PER_DEGREE;
    private final double DEGREES_FOR_SPECIMEN_ON_WALL_TICKS = DEGREES_FOR_SPECIMEN_ON_WALL * ARM_TICKS_PER_DEGREE;

    // Fudge factor for fine control of arm adjustments
    //Larger FudgeFactor = More Jerky Movements
    private static final double FUDGE_FACTOR = 5.0;
    private double armPositionFudgeFactor = 0.0;

    // Arm's current target position
    private double armTargetPosition = GROUND_POSITION_TICKS;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, Constants.HardwareConstants.ARM_MOTOR_NAME);

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure motors
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        setArmPosition(INIT_POSITION_TICKS);  // Set the arm to the ground position by default
    }

    public Action initializeArm() {
        return new InitializeArm();
    }

    public Action raiseArmForWristControl() {
        return new RaiseArmForWristControl();
    }

    public Action deactivateArm() {
        return new DeactivateArm();
    }

    public Action raiseArmForNetzone() {
        return new RaiseArmForNetZone();
    }

    public Action raiseArmForLowerBasket() {
        return new RaiseArmForLowerBasket();
    }

    public Action raiseArmForUpperBasket() {
        return new RaiseArmForHighBasket();
    }

    public Action raiseArmForSamplePickUpFromFloor() {
        return new RaiseArmForSamplePickupFromFloor();
    }

    public Action raiseArmForSpecimenPickUpFromWall() {
        return new RaiseArmForSpecimenPickupFromWall();
    }

    public Action raiseArmForMoving() {
        return new RaiseArmForMoving();
    }

    public Action raiseArmForHighRungHang() {
        return new RaiseArmForSpecimenHangForHighRung();
    }

    public Action moveUpRelativePosition(double deltaToMove) {
        return new MoveArmUpDownRelatively(deltaToMove);
    }


    private void setArmPosition(double targetPosition) {
        // Safety check to ensure position is within valid range
        if (targetPosition < GROUND_POSITION_TICKS || targetPosition > (MAX_POSITION_TICKS+5.0)) {
            targetPosition = LOW_POSITION_TICKS; // Set to low/ground position if out of range
        }

        // Convert target position in ticks (double) and set motor
        armMotor.setTargetPosition((int) targetPosition);  // Motor expects integer target position
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }

    private boolean setArmPositionForAction(TelemetryPacket packet, double targetPosition, double armPower) {

        // Convert target position in ticks (double) and set motor
        armMotor.setTargetPosition((int) targetPosition);  // Motor expects integer target position
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(armPower);

        return false;
    }

    private class MoveArmUpDownRelatively implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;
        private double deltaPosition;

        public MoveArmUpDownRelatively(double deltaToMove) {

            deltaPosition = deltaToMove;
        }

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            double currentPos = armMotor.getCurrentPosition();
            double deltaPositionTicks = deltaPosition * ARM_TICKS_PER_DEGREE;
            double newPosition = currentPos + deltaPositionTicks;
            packet.addLine("currentPos : " + currentPos + " deltaPosition : " + deltaPosition + " deltaPositionTicks : " + deltaPositionTicks + " newPosition : " + newPosition);

            return setArmPositionForAction(packet, newPosition, ARM_POWER);
        }
    }

    private class InitializeArm implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            return setArmPositionForAction(packet, ARM_POSITION_FOR_INIT, ARM_POWER);

        }
    }

    private class ResetArm implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            return false;
        }
    }

    public Action resetArm() {
        return new ResetArm();
    }

    private class RaiseArmForWristControl implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, MIN_POS_FOR_WRIST_TICKS, ARM_POWER);

        }
    }

    private class DeactivateArm implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(RETRACT_ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, ARM_POSITION_FOR_INIT, RETRACT_ARM_POWER);

        }
    }

    private class RaiseArmForNetZone implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, MIN_POS_FOR_WRIST_TICKS, ARM_POWER);

        }
    }
    private class RaiseArmForLowerBasket implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, HIGH_POSITION_TICKS, ARM_POWER);

        }
    }

    private class RaiseArmForHighBasket implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, MAX_POSITION_TICKS, ARM_POWER);

        }
    }

    private class RaiseArmForSpecimenPickupFromWall implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, DEGREES_FOR_SPECIMEN_ON_WALL_TICKS, ARM_POWER);

        }
    }

    private class RaiseArmForSpecimenHangForHighRung implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, ARM_POSITION_FOR_HIGH_RUNG, ARM_POWER);

        }
    }

    private class RaiseArmForSamplePickupFromFloor implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, DEGREES_FOR_SPECIMEN_ON_FLOOR_TICKS, ARM_POWER);

        }
    }

    private class RaiseArmForMoving implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                armMotor.setPower(ARM_POWER);
            }

            // checks lift's current position

            return setArmPositionForAction(packet, DEGREES_FOR_MOVING_HEIGHT_TICKS, ARM_POWER);

        }
    }
}
