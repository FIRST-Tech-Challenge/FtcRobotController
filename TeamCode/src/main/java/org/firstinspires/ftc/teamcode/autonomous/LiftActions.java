package org.firstinspires.ftc.teamcode.autonomous;

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

        rightDrive.setPower(1);
        leftDrive.setPower(1);

        rightDrive.setTargetPosition(rightStartingPos);
        leftDrive.setTargetPosition(leftStartingPos);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public class LiftUp implements Action {
        private boolean initialized = false;
        private int ticks = 3850;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized){
                leftDrive.setTargetPosition(ticks);
                rightDrive.setTargetPosition(ticks);
                initialized = true;
            }
            if (!rightDrive.isBusy() && !leftDrive.isBusy()){
                return false;
            }
            else return true;
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;
        private int ticks = 50;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized){
                leftDrive.setTargetPosition(ticks);
                rightDrive.setTargetPosition(ticks);
                initialized = true;
            }
            if (!rightDrive.isBusy() && !leftDrive.isBusy()){
                return false;
            }
            else return true;
        }
    }

    public Action liftDown() {
        return new LiftDown();
    }

}
