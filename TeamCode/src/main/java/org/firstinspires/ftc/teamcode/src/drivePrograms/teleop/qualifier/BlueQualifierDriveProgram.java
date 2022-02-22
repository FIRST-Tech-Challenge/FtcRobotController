package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.qualifier;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.TripWireDistanceSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

/**
 * Blue Teleop for Qualifier
 */
@Disabled
@TeleOp(name = "Blue Qualifier Drive Program")
public class BlueQualifierDriveProgram extends TeleOpTemplate {
    final BlinkinPattern defaultColor = BlinkinPattern.BLUE;
    private final ElapsedTime yTimer = new ElapsedTime();
    boolean x_depressed = true;
    boolean y_depressed2 = true;
    boolean dPadUpDepressed = true;
    boolean dPadDownDepressed = true;
    int posToGoTo = 0;
    boolean posOn = false;
    BlinkinPattern currentColor = defaultColor;
    TripWireDistanceSensor distanceSensor;

    private Void callBack() {
        this.leds.setPattern(BlinkinPattern.BLACK);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ignored) {
        }
        this.leds.setPattern(currentColor);
        return null;

    }


    public void opModeMain() throws InterruptedException {
        this.initAll();
        distanceSensor = new TripWireDistanceSensor(hardwareMap, "distance_sensor", 8, this::callBack, this::opModeIsActive, this::isStopRequested);
        distanceSensor.start();
        leds.setPattern(defaultColor);

        telemetry.addData("Initialization", "finished");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (posOn) {
                switch (posToGoTo) {
                    case 0:
                        telemetry.addData("Pos", HeightLevel.BottomLevel);
                        break;
                    case 1:
                        telemetry.addData("Pos", HeightLevel.MiddleLevel);
                        break;
                    case 2:
                        telemetry.addData("Pos", HeightLevel.TopLevel);
                        break;
                }
            } else {
                telemetry.addData("Pos", "User Defined");
            }
            telemetry.update();

            //Declan's controls
            {
                driveTrain.setPowerFromGamepad(gamepad1);

                //Declan Speed Modifiers
                if (gamepad1.b) {
                    driveTrain.setDrivePowerMult(0.6);
                }
                if (gamepad1.y) {
                    driveTrain.setDrivePowerMult(1);

                }
                if (gamepad1.a) {
                    driveTrain.setDrivePowerMult(0.3);
                }

                //Declan gamepad y toggle
                if (!gamepad1.x) {
                    x_depressed = true;
                }

            }

            //Eli's controls
            {
                //Handles Linear Slide Control
                {
                    if (posOn) {
                        switch (posToGoTo) {
                            case 0:
                                slide.setTargetLevel(HeightLevel.BottomLevel);
                                break;
                            case 1:
                                slide.setTargetLevel(HeightLevel.MiddleLevel);
                                break;
                            case 2:
                                slide.setTargetLevel(HeightLevel.TopLevel);
                                break;
                        }
                    }


                    if (!gamepad2.dpad_down) {
                        dPadDownDepressed = true;
                    }
                    if (gamepad2.dpad_down && dPadDownDepressed) {
                        dPadDownDepressed = false;
                        posOn = true;
                        posToGoTo--;
                    }

                    if (!gamepad2.dpad_up) {
                        dPadUpDepressed = true;
                    }
                    if (gamepad2.dpad_up && dPadUpDepressed) {
                        dPadUpDepressed = false;
                        posOn = true;
                        posToGoTo++;
                    }

                    if (posToGoTo > 2) {
                        posToGoTo = 2;
                    }
                    if (posToGoTo < 0) {
                        posToGoTo = 0;
                    }


                    if ((gamepad2.left_stick_y) != 0) {
                        slide.setMotorPower(-gamepad2.left_stick_y);
                        posOn = false;
                        //slide.setTargetHeight(slide.getEncoderCount());
                    } else {
                        //slide.threadMain();
                        posOn = false;
                        slide.setMotorPower(0);
                    }
                }
                if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                    intake.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    RevBlinkinLedDriver.BlinkinPattern o = intake.getLEDPatternFromFreight();
                    if (o == null || !intake.isClosed()) {
                        if (currentColor != defaultColor) {
                            leds.setPattern(defaultColor);
                            currentColor = defaultColor;
                        }
                    } else {
                        if (currentColor != o) {
                            leds.setPattern(o);
                            currentColor = o;
                        }
                    }


                } else {
                    intake.setMotorPower(0);
                }

                if (!gamepad2.y) {
                    y_depressed2 = true;
                }
                if (gamepad2.y && y_depressed2) {
                    y_depressed2 = false;
                    if (intake.isClosed()) {
                        intake.setServoOpen();
                        yTimer.reset();
                    } else {
                        intake.setServoClosed();
                    }
                    leds.setPattern(defaultColor);
                    currentColor = defaultColor;
                }

                if (yTimer.seconds() > 1.25) {
                    intake.setServoClosed();
                }

                if (gamepad2.x) {
                    spinner.setPowerBlueDirection();
                } else if (gamepad2.b) {
                    spinner.setPowerRedDirection();
                } else {
                    spinner.stop();
                }


            }
        }
    }
}


