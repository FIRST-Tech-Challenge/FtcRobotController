package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoEncoderTest extends DriveMethods {

    double stateStartTime = -1;

    enum State {
        Unstarted,
        MoveForward,
        Finished
    }

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
                omniDrive(0.5, 0, 0);

                if (robot.leftFrontDrive.getCurrentPosition() >= 1000 * robot.TICKS_PER_MM || getStateTime() >= 10) {
                    // getStateTime is a failsafe
                    omniDrive(0, 0, 0);
                    changeState(State.Finished);
                }
                break;
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }
//spider
    }

    void changeState(State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
    }

    double getStateTime() {
        return getRuntime() - stateStartTime;
    }


//        if (getRuntime() < 2) {
//            omniDrive(0, 1, 0);
//        } else if (getRuntime() < 4) {
//            omniDrive(0, 0, 1);
//        }
//        else {
//                omniDrive(0, 0, 0);
//            }
//        }

    //            case MoveForward:
//                omniDrive(0.5, 0, 0);
//
//                if (getStateTime() >= 0.7) {
//                    omniDrive(0, 0, 0);
//
//                    changeState(State.RaiseArm);
//                }
//                break;
}