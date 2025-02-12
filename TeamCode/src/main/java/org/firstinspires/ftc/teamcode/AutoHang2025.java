package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoHang2025 extends DriveMethods {

    enum State {
        Unstarted,
        MoveForward,
        RaiseArm,
        ExtendSlider,
        ExtraMove,
        ExtraArm,
        OpenClaw,
        Finished
    }
    double stateStartTime = -1;
    double stateStartPos = 0;

    State currentState = State.Unstarted;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("encoder", "%.1f", (double) robot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("state", currentState);
        switch (currentState) {
            case Unstarted:
                changeState(State.MoveForward);
                break;
            case MoveForward:
                double remainingDistance = moveStraightTo(1);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.RaiseArm);
                }
                break;
            case RaiseArm:
                robot.wormGear.setPower(0.5);

                if (robot.wormGearAngle() >= 50) {
                    robot.wormGear.setPower(0);

                    changeState(AutoHang2025.State.ExtendSlider);
                }
                break;
            case ExtendSlider:
                robot.sliderMotor.setPower(1);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(robot.MAX_HORIZONTAL_SLIDER_TICKS);

                if (robot.sliderMotor.getCurrentPosition() >= 2000) {
                    changeState(State.ExtraArm);
                }
                break;
            case ExtraMove:
                omniDrive(0.15, 0, 0);
                break;
            case ExtraArm:
                robot.wormGear.setPower(-0.15);

                if (robot.wormGearAngle() <= 45) {
                    robot.wormGear.setPower(0);

                    changeState(State.OpenClaw);
                }
                break;
            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(AutoHang2025.State.Finished);
                break;
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }
    }
    void changeState(AutoHang2025.State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
        stateStartPos = position();
    }
    double getStateTime() {
        return getRuntime() - stateStartTime;
    }

    double position() {
        double MM_PER_METER = 1000;
        return robot.leftFrontDrive.getCurrentPosition() / robot.TICKS_PER_MM / MM_PER_METER;
    }

    double moveStraightTo(double targetDistance) {
        double distanceTravelled = position();
        double targetPos = stateStartPos + targetDistance;
        double remainingDistance = targetPos - distanceTravelled;
        double MAX_POWER = .5;

        telemetry.addData("remainingDistance", "%.2f", remainingDistance);

        double power = 3 * remainingDistance;

        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        omniDrive(power, 0, 0);

        return remainingDistance;
    }
}
