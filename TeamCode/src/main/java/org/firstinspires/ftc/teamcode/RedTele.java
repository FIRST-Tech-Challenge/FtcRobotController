package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware9010;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

import java.util.Locale;

/**
 * This is the main class that operates the robot
 *
 */
@TeleOp (name="RedTeleOptimized", group="TeleOps")
@Disabled
public class RedTele extends LinearOpMode {

    Hardware9010 hdw;

    MecanumWheels robotWheel;

    @Override
    public void runOpMode() {
        hdw = new Hardware9010(hardwareMap); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels();

        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Encoders.setPosition(1.0);


        hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean SpinnerLeft = false;
        boolean fakeauto = true;
        boolean Turret = false;
        boolean intakeopen = false;
        boolean ManTurretRight;
        boolean ManTurretLeft;
        boolean TargetingRight = false;
        boolean TargetingLeft = false;
        boolean LevelDown = false;
        boolean LevelOff = false;
        boolean IntakeOut;
        boolean highmode = true;
        boolean sharemode = false;
        boolean slowDriveMode = false;
        int steps = 0;
        int currentSlide;
        int currentTurret = 0;
        double powerDrivePercentage = 0.5;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        //hdw.servoRingCounter.setPosition(0.0);


        waitForStart();

        //This is the main loop of operation.
        while (opModeIsActive()) {
            if (gamepad1.start) {
                fakeauto = !fakeauto;
            }
            if (gamepad2.dpad_up) {
                hdw.Encoders.setPosition(1.0);
                sleep(100);
            }
            if (gamepad2.dpad_down) {
                hdw.Encoders.setPosition(0.35);
                sleep(100);
            }
            if (gamepad2.y) {
                highmode = true;
                sharemode = false;
            }
            if (gamepad2.a) {
                highmode = false;
                sharemode = true;
            }

            if (gamepad2.x) {
                SpinnerLeft = !SpinnerLeft;
                sleep(200);
            }
            if (SpinnerLeft) {
                hdw.Spinner.setPower(-1.0);
            } else {
                hdw.Spinner.setPower(0);
            }
            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            float powerTwoRight = gamepad1.right_trigger;
            float powerTwoLeft = gamepad1.left_trigger;

            /* This is important for the driving to work ⬇⬇ */
            hdw.wheelFrontRight.setPower(robotWheel.wheelFrontRightPower * powerDrivePercentage);
            hdw.wheelFrontLeft.setPower(robotWheel.wheelFrontLeftPower * powerDrivePercentage);
            hdw.wheelBackRight.setPower(robotWheel.wheelBackRightPower * powerDrivePercentage);
            hdw.wheelBackLeft.setPower(robotWheel.wheelBackLeftPower * powerDrivePercentage);

        }

    }

}

            /* Turret code has been commented out for driver practice
            if (fakeauto) {  //This controlling code using state machine.
                if (gamepad1.right_bumper) {
                    if (steps == 3) {
                        steps = steps + 2;
                        sleep(100);
                    } else if (steps == 20 || steps == 21) {
                        steps = 0;
                        sleep(100);
                    } else {
                        steps = steps + 1;
                        sleep(100);
                    }
                }
                if (gamepad1.left_bumper) {
                    if (steps == 5) {
                        steps = steps - 2;
                        sleep(100);
                    } else if (steps == 20 || steps == 21) {
                        steps = 0;
                        sleep(100);
                    } else {
                        steps = steps - 1;
                        sleep(100);
                    }
                }

                if (steps == 0) {
                    hdw.grabberclaw.setPosition(0.8);
                    intakeopen = true;
                    hdw.Turret.setTargetPosition(0);
                    hdw.Vertical.setTargetPosition(0);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right
                            || gamepad1.dpad_left || gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2) {
                        steps = 21;
                    }
                } else if (steps == 1) {
                    hdw.grabberclaw.setPosition(1.0);
                    intakeopen = false;
                    sleep(50);
                    hdw.Turret.setTargetPosition(0);
                    hdw.Vertical.setTargetPosition(0);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.b) {
                        steps = 20;
                    }
                    if (hdw.sensorDistance.getDistance(DistanceUnit.CM) > 3.5) {
                        steps = 0;
                    }
                } else if (steps == 2) {
                    hdw.grabberclaw.setPosition(1.0);
                    sleep(50);
                    hdw.Turret.setTargetPosition(0);
                    hdw.Vertical.setTargetPosition(600);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.b) {
                        steps = 20;
                    }
                } else if (steps == 3 && hdw.Turret.getCurrentPosition() > -150 && highmode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Turret.setTargetPosition(-500);
                    hdw.Vertical.setTargetPosition(1150);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(-1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.left_trigger > 0.2) {
                        steps = 4;
                    }
                    if (gamepad1.right_trigger > 0.2) {
                        steps = 4;
                    }
                } else if (steps == 3 && hdw.Turret.getCurrentPosition() < -150 && highmode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Turret.setTargetPosition(-500);
                    hdw.Vertical.setTargetPosition(1150);
                    hdw.Slide.setTargetPosition(2500);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(-1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.left_trigger > 0.2) {
                        steps = 4;
                    }
                    if (gamepad1.right_trigger > 0.2) {
                        steps = 4;
                    }
                } else if (steps == 3 && hdw.Turret.getCurrentPosition() < 150 && sharemode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Turret.setTargetPosition(200);
                    hdw.Vertical.setTargetPosition(750);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.left_trigger > 0.2) {
                        steps = 4;
                    }
                    if (gamepad1.right_trigger > 0.2) {
                        steps = 4;
                    }
                } else if (steps == 3 && hdw.Turret.getCurrentPosition() > 150 && sharemode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Turret.setTargetPosition(200);
                    hdw.Vertical.setTargetPosition(750);
                    hdw.Slide.setTargetPosition(500);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                    if (gamepad1.left_trigger > 0.2) {
                        steps = 4;
                    }
                    if (gamepad1.right_trigger > 0.2) {
                        steps = 4;
                    }
                    if (gamepad1.dpad_right) {
                        steps = 4;
                    }
                    if (gamepad1.dpad_left) {
                        steps = 4;
                    }
                } else if (steps == 4 && highmode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Vertical.setTargetPosition(1150);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (gamepad1.left_trigger > 0.2) {
                        if (hdw.Slide.getCurrentPosition() > 200) {
                            hdw.Slide.setPower(-powerTwoLeft);
                        } else {
                            hdw.Slide.setPower(0.0);
                        }
                    } else {
                        if (gamepad1.right_trigger > 0.2) {
                            if (hdw.Slide.getCurrentPosition() < 3500) {
                                hdw.Slide.setPower(powerTwoRight);
                            }
                        } else {
                            hdw.Slide.setPower(0.0);
                        }
                    }
                    if (gamepad1.dpad_right) {
                        hdw.Turret.setPower(0.4);
                    } else if (gamepad1.dpad_left) {
                        hdw.Turret.setPower(-0.4);
                    } else {
                        hdw.Turret.setPower(0.0);
                    }
                } else if (steps == 4 && sharemode) {
                    hdw.grabberclaw.setPosition(1.0);
                    hdw.Vertical.setTargetPosition(750);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (gamepad1.left_trigger > 0.2) {
                        if (hdw.Slide.getCurrentPosition() > 200) {
                            hdw.Slide.setPower(-powerTwoLeft);
                        } else {
                            hdw.Slide.setPower(0.0);
                        }
                    } else {
                        if (gamepad1.right_trigger > 0.2) {
                            if (hdw.Slide.getCurrentPosition() < 3500) {
                                hdw.Slide.setPower(powerTwoRight);
                            }
                        } else {
                            hdw.Slide.setPower(0.0);
                        }
                    }
                    if (gamepad1.dpad_right) {
                        hdw.Turret.setPower(0.65);
                    } else if (gamepad1.dpad_left) {
                        hdw.Turret.setPower(-0.65);
                    } else {
                        hdw.Turret.setPower(0.0);
                    }
                } else if (steps == 5 && highmode) {
                    hdw.grabberclaw.setPosition(0.7);
                    currentTurret = hdw.Turret.getCurrentPosition();
                    hdw.Turret.setTargetPosition(currentTurret);
                    hdw.Vertical.setTargetPosition(1150);
                    currentSlide = hdw.Slide.getCurrentPosition();
                    hdw.Slide.setTargetPosition(currentSlide);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                } else if (steps == 5 && sharemode) {
                    hdw.grabberclaw.setPosition(0.7);
                    currentTurret = hdw.Turret.getCurrentPosition();
                    hdw.Turret.setTargetPosition(currentTurret);
                    hdw.Vertical.setTargetPosition(750);
                    currentSlide = hdw.Slide.getCurrentPosition();
                    hdw.Slide.setTargetPosition(currentSlide);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(-1);
                    hdw.Slide.setPower(1);
                } else if (steps == 6 && hdw.Slide.getCurrentPosition() > 1500) {
                    hdw.Slide.setTargetPosition(0);
                    hdw.Turret.setTargetPosition(currentTurret);
                    hdw.Vertical.setTargetPosition(1150);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Slide.setPower(1);
                    hdw.Turret.setPower(1);
                } else if (steps == 6 && hdw.Slide.getCurrentPosition() < 1500) {
                    hdw.Slide.setTargetPosition(0);
                    hdw.Turret.setTargetPosition(0);
                    hdw.Vertical.setTargetPosition(750);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Slide.setPower(1);
                    hdw.Turret.setPower(1);
                } else if (steps == 20) {
                    hdw.grabberclaw.setPosition(0.7);
                    intakeopen = false;
                    hdw.Turret.setTargetPosition(0);
                    hdw.Vertical.setTargetPosition(0);
                    hdw.Slide.setTargetPosition(0);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hdw.Vertical.setPower(1);
                    hdw.Turret.setPower(1);
                    hdw.Slide.setPower(1);
                } else if (steps == 21) {
                    if (gamepad1.dpad_down) {
                        hdw.Vertical.setPower(-0.3);
                        hdw.Vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else if (gamepad1.dpad_up) {
                        hdw.Vertical.setPower(0.3);
                        hdw.Vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else {
                        hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                    if (gamepad1.dpad_left) {
                        hdw.Turret.setPower(-0.5);
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else if (gamepad1.dpad_right) {
                        hdw.Turret.setPower(0.5);
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else {
                        hdw.Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                    if (gamepad1.right_trigger > 0.2) {
                        hdw.Slide.setPower(0.5);
                        hdw.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else if (gamepad1.left_trigger > 0.2) {
                        hdw.Slide.setPower(-0.5);
                        hdw.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        sleep(50);
                    } else {
                        hdw.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                } else if (hdw.Slide.getCurrentPosition() > 1500) {
                    steps = 6;
                } else {
                    steps = 0;
                }

                if (hdw.sensorDistance.getDistance(DistanceUnit.CM) < 2.5 && intakeopen) {
                    hdw.grabberclaw.setPosition(1.0);
                    sleep(100);
                    steps = 1;
                }

            } else {  // This is controlling without using state machine.
                if (gamepad1.right_bumper) {
                    Turret = true;
                    TargetingRight = !TargetingRight;
                    TargetingLeft = false;
                    sleep(200);
                }
                if (gamepad1.left_bumper) {
                    Turret = true;
                    TargetingLeft = !TargetingLeft;
                    TargetingRight = false;
                    sleep(200);
                }
                if (gamepad1.dpad_left) {
                    Turret = false;
                    TargetingRight = false;
                    TargetingLeft = false;
                    ManTurretLeft = true;
                } else {
                    ManTurretLeft = false;
                }
                if (gamepad1.dpad_right) {
                    Turret = false;
                    TargetingRight = false;
                    TargetingLeft = false;
                    ManTurretRight = true;
                } else {
                    ManTurretRight = false;
                }

                if (Turret) {
                    if (TargetingRight) {
                        hdw.Turret.setTargetPosition(550);
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hdw.Turret.setPower(1);
                    } else if (TargetingLeft) {
                        hdw.Turret.setTargetPosition(-550);
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hdw.Turret.setPower(-1);
                    } else {
                        hdw.Turret.setTargetPosition(0);
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hdw.Turret.setPower(1);
                    }
                } else {
                    if (ManTurretRight) {
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hdw.Turret.setPower(0.3);
                    } else if (ManTurretLeft) {
                        hdw.Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hdw.Turret.setPower(-0.3);
                    } else {
                        hdw.Turret.setPower(0.0);
                    }
                }


                if (gamepad1.left_trigger > 0.2) {
                    hdw.Slide.setPower(-powerTwoLeft);
                } else {
                    if (gamepad1.right_trigger > 0.2) {
                        if (hdw.Slide.getCurrentPosition() < 3500) {
                            hdw.Slide.setPower(powerTwoRight);
                        } else {
                            hdw.Slide.setPower(0.0);
                        }
                    } else {
                        hdw.Slide.setPower(0.0);
                    }
                }

                //Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                //        (int) (sensorColor.green() * SCALE_FACTOR),
                //        (int) (sensorColor.blue() * SCALE_FACTOR),
                //        hsvValues);

                if (gamepad2.dpad_up) {
                    highmode = true;
                    sharemode = false;
                }
                if (gamepad2.dpad_down) {
                    sharemode = true;
                    highmode = false;
                }

                if (gamepad1.x) {
                    LevelDown = !LevelDown;
                    sleep(200);
                }
                if (gamepad1.y) {
                    sleep(200);
                }
                if (gamepad1.dpad_up) {
                    sleep(200);
                }
                if (gamepad1.dpad_down) {
                    sleep(200);
                }

                if (LevelDown) {
                    hdw.Vertical.setTargetPosition(650);
                    hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //hdw.Vertcial.setMaxPower(5000);
                    hdw.Vertical.setPower(-1.0);
                }
                //else if (hdw.liftsensor.isPressed())
                //{
                //    hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //hdw.Vertical.setTargetPosition(-100);
                //hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //hdw.Vertical.setPower(-0.2);


                //}
                // Alex - bug was here
                //hdw.Vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //hdw.Vertical.setPower(-0.6);

                if (gamepad1.dpad_left)
                {
                    LevelDown = false;
                    LevelOff = !LevelOff;
                    sleep(200);
                }



                if (gamepad1.back) {
                    slowDriveMode = !slowDriveMode;
                    sleep(200);
                }
                if (slowDriveMode) {
                    powerDrivePercentage = 0.5;
                } else {
                    powerDrivePercentage = 1;
                }

                if (gamepad1.a) {
                    sleep(200);
                } else {
                    sleep(100);
                }

                if (gamepad1.b) {
                    IntakeOut = true;
                    sleep(200);
                } else {
                    IntakeOut = false;
                    sleep(100);
                }

                if (IntakeOut) {
                    hdw.grabberclaw.setPosition(0.7);
                } else {
                    if (hdw.sensorDistance.getDistance(DistanceUnit.CM) < 2) {
                        hdw.grabberclaw.setPosition(1.0);
                    }
                }
            }

                telemetry.addData("count", steps);
                if (highmode) {
                    telemetry.addData("HIGH_mode", highmode);
                }
                if (sharemode) {
                    telemetry.addData("SHARE_mode", sharemode);
                }

                telemetry.addData("SlidePower", hdw.Slide.getPower());
                telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", hdw.sensorDistance.getDistance(DistanceUnit.CM)));
                //telemetry.addData("SlideTemp", currentSlide);
                telemetry.addData("Alpha", hdw.sensorColor.alpha());
                telemetry.addData("Red", hdw.sensorColor.red());
                //telemetry.addData("liftsensor", hdw.liftsensor.isPressed());
                telemetry.addData("Green", hdw.sensorColor.green());
                telemetry.addData("Blue", hdw.sensorColor.blue());
                //telemetry.addData("Hue", hsvValues[0]);
                telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
                telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
                telemetry.addData("strafeWheel", hdw.wheelStrafe.getCurrentPosition());
                telemetry.addData("TurretDistance", hdw.Turret.getCurrentPosition());
                telemetry.addData("VerticalDistance", hdw.Vertical.getCurrentPosition());
                telemetry.addData("VerticalSpeed", hdw.Vertical.getPower());
                telemetry.addData("SlideDistance", hdw.Slide.getCurrentPosition());
                telemetry.addData("LeftTrigger", gamepad1.left_trigger);
                telemetry.addData("RightTrigger", gamepad1.right_trigger);
                telemetry.addData("LeftPower", powerTwoLeft);
                telemetry.addData("RightPower", powerTwoRight);

                telemetry.addData("turbo", robotWheel.turbo);
                //telemetry.addData("Max", hdw.Vertical.getSpeed());

                telemetry.update();

            }
        }
    }

}
             */
