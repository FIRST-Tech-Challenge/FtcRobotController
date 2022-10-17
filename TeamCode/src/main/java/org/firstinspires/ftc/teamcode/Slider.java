package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slider {
    private Button ctrlUp;
    private Button ctrlDown;
    private Button level1Ctrl;
    private Button level2Ctrl;
    private Button level3Ctrl;
    private Button intakeCtrl;
    private DcMotorEx motor;
    private Telemetry telemetry;

    private enum State {
        PARK, IN_POSITION, LEVEL1, LEVEL2, LEVEL3;
    }

    private State state = State.PARK;

    private int[] levelPositions = {0, 200, 533, 1121, 1678};
    private int levelSelected = 0;
    private boolean runToLevel = false;

    public Slider(HardwareMap hardwareMap, Telemetry telemetry, ExtendedGamepad gamepad2) {
        this.ctrlUp = gamepad2.dpad_up;
        this.ctrlDown = gamepad2.dpad_down;
        this.level1Ctrl = gamepad2.x;
        this.level2Ctrl = gamepad2.y;
        this.level3Ctrl = gamepad2.b;
        this.intakeCtrl = gamepad2.right_bumper;
        this.motor = hardwareMap.get(DcMotorEx.class, "slider");
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.telemetry = telemetry;
    }

    public void setSliderLevel(int level) {
        motor.setTargetPosition(levelPositions[level + 1]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(1600);
    }

    public void run(Intake intake, Bucket bucket) {
        if (level1Ctrl.isBumped()) {
            levelSelected = 1;
            runToLevel = true;
        } else if (level2Ctrl.isBumped()) {
            levelSelected = 2;
            runToLevel = true;
        } else if (level3Ctrl.isBumped()) {
            levelSelected = 3;
            runToLevel = true;
        } else if (intakeCtrl.isBumped()) {
            levelSelected = -1;
            runToLevel = true;
        }

        if (runToLevel) {
            if (levelSelected == -1) {
                if (intake.isRunning()) {
                    bucket.setIntake();
                    setSliderLevel(-1);
                } else {
                    setSliderLevel(0);
                    bucket.setVertical();
                    if (!motor.isBusy()) runToLevel = false;
                }
            } else {
                if (state == State.PARK) {
                    setSliderLevel(levelSelected);
                    state = State.IN_POSITION;
                } else if (state == State.IN_POSITION && (
                        (level1Ctrl.isBumped() && levelSelected == 1) ||
                                (level2Ctrl.isBumped() && levelSelected == 2) ||
                                (level3Ctrl.isBumped() && levelSelected == 3)
                )
                ) {
                    setSliderLevel(0);
                    if (!motor.isBusy()) {
                        state = State.PARK;
                        runToLevel = false;
                    }
                }
            }
        } else if (ctrlUp.isPressed()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.5);
        } else if (ctrlDown.isPressed()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        telemetry.addData("Encoder Value:", motor.getCurrentPosition());
    }
}
