package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBasket2024 extends DriveMethods {

    double stateStartTime = -1;
    double stateStartPos = 0;


    enum State {
        Finished,
        Unstarted,
        TightenClaw,
        StrafeRight,
        Wait,
        RaiseArm,
        ExtendSlider,
        MoveForward,
        OpenClaw,
        WaitThree,
        WaitTwo,
        MoveBackward,
        ExtraMove,
        LowerSlider
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
                changeState(State.TightenClaw);
                break;
            case TightenClaw:
                robot.clawServo.setPosition(1.20);
                changeState(State.StrafeRight);
                break;
            case StrafeRight:
                double remainingDistance = moveStraightTo(.325);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.Wait);
                }
                break;
            case Wait:
                if (getStateTime() >= 4) {
                    omniDrive(0, 0, 0);
                    changeState(State.MoveForward);
                }
            case MoveForward:
                double remainingPos = moveStraightTwo(0.175);

                if (Math.abs(remainingPos) <= .01 || remainingPos < 0) {
                    omniDrive(0, 0, 0);
                    changeState(State.WaitTwo);
                }
                break;
            case WaitTwo:
                if (getStateTime() >= 4) {
                    omniDrive(0, 0, 0);
                    changeState(State.RaiseArm);
                }
            case RaiseArm:
                robot.wormGear.setPower(0.5);

                if (robot.wormGearAngle() >= 78) {
                    robot.wormGear.setPower(0);

                    changeState(State.ExtendSlider);
                }
                break;
            case ExtendSlider:
                robot.sliderMotor.setPower(0.5);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(2700);

                if (robot.sliderMotor.getCurrentPosition() >= 2700) {
                    changeState(State.ExtraMove);
                }
                break;
            case ExtraMove:
                remainingPos = moveStraightTwo(0.175);

                if (Math.abs(remainingPos) <= .01 || remainingPos < 0) {
                    omniDrive(0, 0, 0);
                    changeState(State.OpenClaw);
                }
                break;
            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(State.WaitThree);
                break;
            case WaitThree:
                if (getStateTime() > 5) {
                    changeState(State.MoveBackward);
                }
            case MoveBackward:
                double remaining = moveStraightTwo(-0.5);

                if (Math.abs(remaining) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.LowerSlider);
                    }
                break;
            case LowerSlider:
                robot.sliderMotor.setPower(0.5);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(11);

                if (robot.sliderMotor.getCurrentPosition() <= 11) {
                    changeState(State.Finished);
                }
            case Finished:
                robot.sliderMotor.setPower(0);
                moveStraightTo(0);
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

        omniDrive(0, power, 0);

        return remainingDistance;
    }

    double moveStraightTwo(double targetPos) {
        double distanceTravelled = position();
        double targetPosi = stateStartPos + targetPos;
        double remainingPos = targetPosi - distanceTravelled;
        double MAX_POWER = .5;

        telemetry.addData("remainingDistance", "%.2f", remainingPos);

        double power = 3 * remainingPos;

        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        omniDrive(power, 0, 0);

        return remainingPos;

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
}
