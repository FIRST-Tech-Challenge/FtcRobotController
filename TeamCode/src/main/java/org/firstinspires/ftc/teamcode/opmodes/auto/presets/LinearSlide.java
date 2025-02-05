package org.firstinspires.ftc.teamcode.opmodes.auto.presets;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.opmodes.Subsystems.PresetSlideCode;

public class LinearSlide {

    public final DcMotorEx linearSlideMotor;
    private static final double LINEAR_SLIDE_POWER = 0.5;

    private static final double RETRACT_LINEAR_SLIDE_POWER = -0.5;
    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 14.0;
    private static final double EXTEND_FULL_DEGREES = 2000.0;     // Position to place into an high basket (70 degrees)

    private static final double RETRACT_FULL_DEGREES = 0.0;

    private static final double EXTEND_HALF_DEGREES = 800.0;

    private static final double EXTEND_HIGH_RUNG_DEGREES = 220.0;

    private static final double EXTEND_SLIDE_FOR_PICKUP_FROM_FLOOR_DEGREES = 50.0;

    private static final double MAX_LENGTH = 1800;

    private static final double MIN_LENGTH = 5.0;

    // Formula to calculate ticks per degree
    final double LINEAR_SLIDE_TICKS_PER_DEGREE = 19.2032086;
//            145.1 // encoder ticks per rotation of the bare RS-555 motor
//                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
//                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
//                    * 1 / 360.0 * 2; // we want ticks per degree, not per rotation


    // Pre-calculated arm positions in encoder ticks based on degrees
    private final double INIT_POSITION_TICKS = INIT_DEGREES* LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double EXTEND_SLIDE_FOR_PICKUP_FROM_FLOOR_DEGREES_TICKS = EXTEND_SLIDE_FOR_PICKUP_FROM_FLOOR_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double EXTEND_FULL_DEGREES_TICKS = EXTEND_FULL_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double RETRACT_FULL_DEGREES_TICKS = RETRACT_FULL_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double EXTEND_HALF_DEGREES_TICKS = EXTEND_HALF_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;

    // Fudge factor for fine control of arm adjustments
    //Larger FudgeFactor = More Jerky Movements
    private static final double FUDGE_FACTOR = 5.0;


    public LinearSlide(HardwareMap hardwareMap) {

        linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, Constants.HardwareConstants.LINEAR_SLIDE_MOTOR_NAME);

        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        linearSlideMotor.setPositionPIDFCoefficients(1.0);
        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public Action initLinearSlide() {
        return new InitLinearSlide();
    }

    public Action extendArmForward() {
        return new ExtendLinearSlide();
    }

    public Action extendArmHalfway() {
        return new ExtendSlideHalfLength();
    }

    public Action retractSlideBackward() {
        return new RetractSlideBackward();
    }

    public Action extendSlideForPickFromPool() {
        return new ExtendSlideForPickFromPool();
    }

    public Action moveSlideRelatively(double delta) {
        return new MoveSlideForwardOrBackWardRelative(delta);
    }

    public Action moveSlideForHighRung() {
        return new ExtendSlideForHighRung();
    }

    public Action resetLinearSlide() {
        return new ResetSlidePosition();
    }

    public class MoveSlideForwardOrBackWardRelative implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        private double deltaPos;

        public MoveSlideForwardOrBackWardRelative(double deltaPosition) {

            deltaPos = deltaPosition;
        }

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }

            double currentPos = linearSlideMotor.getCurrentPosition();
//            double deltaPosition = ;
            double newPosition = currentPos + deltaPos;
            packet.addLine("currentPos : " + currentPos + " deltaPosition : " + deltaPos + " newPosition : " + newPosition);

            return setLinearSlidePosition(packet, newPosition, LINEAR_SLIDE_POWER);

        }
    }

    public class InitLinearSlide implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }

            
            return setLinearSlidePosition(packet, INIT_DEGREES, LINEAR_SLIDE_POWER);

        }
    }


    public class ExtendLinearSlide implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }

            return setLinearSlidePosition(packet, EXTEND_FULL_DEGREES, LINEAR_SLIDE_POWER);

        }
    }

    public class RetractSlideBackward implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(RETRACT_LINEAR_SLIDE_POWER);
            }

            return setLinearSlidePosition(packet, RETRACT_FULL_DEGREES, RETRACT_LINEAR_SLIDE_POWER);

        }
    }

    public class ExtendSlideForPickFromPool implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }

            return setLinearSlidePosition(packet, EXTEND_SLIDE_FOR_PICKUP_FROM_FLOOR_DEGREES, LINEAR_SLIDE_POWER);

        }
    }

    public class ExtendSlideForHighRung implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }


            return setLinearSlidePosition(packet, EXTEND_HIGH_RUNG_DEGREES, LINEAR_SLIDE_POWER);

        }
    }

    public class ExtendSlideHalfLength implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                initialized = true;
//                linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
            }

            
            return setLinearSlidePosition(packet, EXTEND_HALF_DEGREES, LINEAR_SLIDE_POWER);

        }
    }

    private boolean setLinearSlidePosition(TelemetryPacket packet, double targetPosition, double linearSlidePower) {

        // Convert target position in ticks (double) and set motor
        linearSlideMotor.setTargetPosition((int) targetPosition);
        linearSlideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(linearSlidePower);

        return false;

    }

    private class ResetSlidePosition implements Action {
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

            linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            return false;
        }
    }
}
