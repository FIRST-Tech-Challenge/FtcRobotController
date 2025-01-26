package org.firstinspires.ftc.teamcode.subsystems.frontarm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.lift.LiftActions;

public class FrontArmActions {
    private DcMotor frontDrive;
    private DcMotor spinner;

    private Servo spinnerRotator;
    private Servo slopeRotator;

    int startingPos;

    public FrontArmActions(HardwareMap hardwareMap) {
        frontDrive = hardwareMap.get(DcMotor.class, "frontArm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        startingPos = frontDrive.getCurrentPosition();

    }

    public class ExtendArm implements Action {
        private boolean initialized = false;
        private int ticks = 1000; // TODO Record Tick Amount 1000 is place holder

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized){
                frontDrive.setPower(1);
                frontDrive.setTargetPosition(ticks);
                frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }
            return frontDrive.isBusy();
        }
    }

    public Action extendArm() {
        return new ExtendArm();
    }

    /*TODO Add retract arm Action
    *  Add servo control for each to align the spinner and slope*/

    public class RetractArm implements Action {
        private boolean initialized = false;
        private int ticksSafeZone = 50; // TODO Record Tick Amount 1000 is place holder

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized){
                frontDrive.setPower(1);
                frontDrive.setTargetPosition(ticksSafeZone + startingPos);
                frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                initialized = true;
            }
            return frontDrive.isBusy();
        }
    }

    public Action retractArm() {
        return new RetractArm();
    }
}
