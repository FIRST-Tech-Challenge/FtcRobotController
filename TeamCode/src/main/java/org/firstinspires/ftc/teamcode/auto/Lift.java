package org.firstinspires.ftc.teamcode.auto;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Lift Class - perform up or down position of the lift.
 */
public class Lift {
    private DcMotorEx liftMotor1 = null, liftMotor2 = null;

    /**
     * LiftUp class implements Action to move the lift to specified up position
     */
    public class LiftUp implements Action{
        private boolean initialized = false;
        private double targetPosition = 0.0;
        public LiftUp(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // powers on motor, if it is not on
            if (!initialized) {
                liftMotor1.setPower(1.0);
                liftMotor2.setPower(1.0);
                initialized = true;
            }

            double liftMotor1Pos = liftMotor1.getCurrentPosition();
            double liftMotor2Pos = liftMotor2.getCurrentPosition();
            telemetryPacket.put("LiftUp Motor1 Position ", liftMotor1Pos);
            telemetryPacket.put("LiftUp Motor2 Position ", liftMotor2Pos);

            if (liftMotor1Pos < targetPosition || liftMotor2Pos < targetPosition) {
                return true;
            } else {
                liftMotor1.setPower(0);
                liftMotor2.setPower(0);
                return false;
            }
        }
    }

    /**
     * LitDown class implements Action to run the lift to specified down position
     */
    public class LiftDown implements Action {
        private boolean initialized = false;
        private double targetPosition = 0.0;

        public LiftDown(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                liftMotor1.setPower(-0.8);
                liftMotor2.setPower(-0.8);
                initialized = true;
            }

            double liftMotor1Pos = liftMotor1.getCurrentPosition();
            double liftMotor2Pos = liftMotor2.getCurrentPosition();
            telemetryPacket.put("LiftDown Motor1 Position ", liftMotor1Pos);
            telemetryPacket.put("LiftDown Motor2 Position ", liftMotor2Pos);

            if (liftMotor1Pos > targetPosition || liftMotor2Pos > targetPosition) {
                return true;
            } else {
                liftMotor1.setPower(0);
                liftMotor2.setPower(0);
                return false;
            }
        }
    }

    /**
     * Lift Constructor
     * @param hardwareMap
     */
    public Lift(HardwareMap hardwareMap) {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Create a new LiftUp Action
     * @param targetPosition
     * @return
     */
    public Action liftUp(double targetPosition) {
        return new LiftUp(targetPosition);
    }

    /**
     * Create a new LiftDown Action
     * @param targetPosition
     * @return
     */
    public Action liftDown(double targetPosition) {
        return new LiftDown(targetPosition);
    }
}