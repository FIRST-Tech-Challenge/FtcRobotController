package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoPork extends DriveMethods {

    enum State {
        Unstarted,
        MoveForward,
        MoveRight,
        ExtraForward,
        ExtraRight,
        MoveBackward,
        LastForward,
        MoveLeft,
        Finished,
    }

    double stateStartTime = 0;
    AutoPork.State currentState = AutoPork.State.Unstarted;

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
        omniDrive(0.15,0,0);
        if (getStateTime() >=2);
        omniDrive(0,0,0);
        changeState(State.MoveRight);
        break;
    case MoveRight:
        omniDrive(0,0.15,0);
        if (getStateTime() >= 4);
        omniDrive(0,0,0);
        changeState(State.ExtraForward);
        break;
    case ExtraForward:
        omniDrive(0.15,0,0);
        if(getStateTime() >= 5);
        omniDrive(0, 0, 0);
        changeState(State.ExtraRight);
        break;
    case ExtraRight:
        omniDrive(0, 0.15,0);
        if(getStateTime() >= 6);
        omniDrive(0, 0, 0);
        changeState(State.MoveBackward);
        break;
    case MoveBackward:
        omniDrive(-0.15, 0,0);
        if(getStateTime() >= 7);
        omniDrive(0, 0, 0);
        changeState(State.LastForward);
        break;
    case LastForward:
        omniDrive(0.15, 0,0);
        if(getStateTime() >= 8);
        omniDrive(0, 0, 0);
        changeState(State.MoveLeft);
        break;
    case MoveLeft:
        omniDrive(0, -0.15,0);
        if(getStateTime() >= 9);
        omniDrive(0, 0, 0);
        changeState(State.Finished);
        break;
    case Finished:
        omniDrive(0,0,0);
        break;
}



    }
    void changeState(AutoPork.State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
    }
    double getStateTime() {
        return getRuntime() - stateStartTime;
    }
}
