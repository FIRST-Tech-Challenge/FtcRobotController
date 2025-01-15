package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBasket2024 extends DriveMethods {

    double startTime = -1;

    enum State {
        Sideways,
        Turning,
        Finished,
        Unstarted,
        RaiseArm,
        ExtendSlider,
        OpenClaw
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
        telemetry.addData("sliderLength","%.1f", (double) robot.sliderMotor.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());

        telemetry.addData("state", currentState);

        switch (currentState) {
            case Unstarted:
                currentState = State.RaiseArm;
                break;
            case RaiseArm:
                robot.wormGear.setPower(0.5);

                if (robot.wormGearAngle() >= 50) {
                    robot.wormGear.setPower(0);
                    currentState = State.ExtendSlider;
                }
                break;
            case ExtendSlider:
                robot.sliderMotor.setPower(1);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(2000);

                if (robot.sliderMotor.getCurrentPosition() >= 2000) {
                    currentState = State.OpenClaw;
                }
                break;
            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                currentState = State.Finished;
                break;
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }

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
    }