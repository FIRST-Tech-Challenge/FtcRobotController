package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoPark extends DriveMethods {

    enum State {
        Unstarted,
        MoveForward,
        MoveRight,
        ExtraForward,
        ExtraRight,
        MoveBackward,
        Finished,
    }

    double stateStartTime = 0;
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
                omniDrive(0.15, 0, 0);
                if (getStateTime() >= 1) {
                    omniDrive(0, 0, 0);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    changeState(State.MoveRight);
                }
                break;
            case MoveRight:
                omniDrive(0, 0.15, 0);
                if (getStateTime() >= 1.8) {
                    omniDrive(0, 0, 0);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    changeState(State.ExtraForward);
                }
                break;
            case ExtraForward:
                omniDrive(0.15, 0, 0);
                if (getStateTime() >= 5.7) {
                    omniDrive(0, 0, 0);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    changeState(State.ExtraRight);
                }
                break;
            case ExtraRight:
                omniDrive(0, 0.15, 0);
                if (getStateTime() >= 1.8) {
                    omniDrive(0, 0, 0);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    changeState(State.MoveBackward);
                }
                break;
            case MoveBackward:
                omniDrive(-0.15, 0, 0);
                if (getStateTime() >= 5.5) {
                    omniDrive(0, 0, 0);

                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    changeState(State.Finished);
                }
                break;
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }


    }

    void changeState(State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
    }

    double getStateTime() {

        return getRuntime() - stateStartTime;
    }
}
