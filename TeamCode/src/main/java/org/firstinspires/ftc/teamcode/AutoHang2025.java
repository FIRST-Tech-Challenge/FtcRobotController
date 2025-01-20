package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoHang2025 extends DriveMethods {

    enum State {
        Unstarted,
        MoveForward,
        RaiseArm,
        ExtendSlider,
        ExtraArm,
        ExtraSlider,
        OpenClaw,
        Finished
    }
    double stateStartTime = -1;

    State currentState = State.Unstarted;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("sliderLength", "%.1f", (double) robot.sliderMotor.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());

        telemetry.addData("state", currentState);
        switch (currentState) {
            case Unstarted:
                changeState(State.MoveForward);
                break;
            case MoveForward:
                omniDrive(0.5, 0, 0);

                if (getStateTime() >= 0.7) {
                    omniDrive(0, 0, 0);

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
            case ExtraArm:
                robot.wormGear.setPower(-0.15);

                if (robot.wormGearAngle() <= 45) {
                    robot.wormGear.setPower(0);

                    changeState(AutoHang2025.State.ExtraSlider);
                }
                break;
            case ExtraSlider:
                changeState(State.OpenClaw);
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
    }
    double getStateTime() {
        return getRuntime() - stateStartTime;
    }
}
