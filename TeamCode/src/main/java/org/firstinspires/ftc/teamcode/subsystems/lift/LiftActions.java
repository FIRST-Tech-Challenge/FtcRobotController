package org.firstinspires.ftc.teamcode.subsystems.lift;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftActions {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    int rightStartingPos;
    int leftStartingPos;

    public LiftActions(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "armLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "armRight");

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightStartingPos = rightDrive.getCurrentPosition();
        leftStartingPos = leftDrive.getCurrentPosition();

    }

    public class LiftUp implements Action {
        private boolean initialized = false;
        private int ticks = 2750;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftDrive.setPower(1);
                rightDrive.setPower(1);

                leftDrive.setTargetPosition(ticks);
                rightDrive.setTargetPosition(ticks);

                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }
            return rightDrive.isBusy() || leftDrive.isBusy();
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;
        private int ticksSafeZone = 50;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftDrive.setPower(1);
                rightDrive.setPower(1);

                leftDrive.setTargetPosition(ticksSafeZone + leftStartingPos);
                rightDrive.setTargetPosition(ticksSafeZone + rightStartingPos);

                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }
            // returns if the motors are still moving
            return rightDrive.isBusy() || leftDrive.isBusy();
        }
    }

    public Action liftDown() {
        return new LiftDown();
    }

}
