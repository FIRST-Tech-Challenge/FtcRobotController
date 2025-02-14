package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoEncoderTest extends DriveMethods {

    double stateStartTime = -1;
    double stateStartPos = 0;

    enum State {
        Unstarted,
        MoveForward,
        Wait,
        Move2,
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
                double remainingDistance = moveStraightToo(1);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.Wait);
                }
                break;
                case Wait:
                    moveStraightToo(0);
                    if (getStateTime() >= 2) {
                        changeState(State.Move2);
                    }
                    break;
                case Move2:
                    remainingDistance = moveStraightToo(-1);
                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.Finished);
                }
                break;
            case Finished:

                moveStraightToo(0);
                break;
        }
    }

    void changeState(State nextState) {
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

    double moveStraightToo(double targetDistance) {
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