package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.old.state;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.TripWireDistanceSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@Disabled
@TeleOp(name = "\uD83D\uDFE6 Blue State Drive Program \uD83D\uDFE6")
public class BlueStateDriveProgram extends TeleOpTemplate {
    final BlinkinPattern defaultColor = BlinkinPattern.BLUE;
    private final ElapsedTime yTimer = new ElapsedTime();

    private final ElapsedTime slideResetTimer = new ElapsedTime();
    boolean y_depressed2 = true;
    boolean dPadUpDepressed = true;
    boolean dPadDownDepressed = true;
    boolean manualSlideControl = false;
    HeightLevel currentLevel = HeightLevel.Down;
    BlinkinPattern currentColor = defaultColor;
    TripWireDistanceSensor distanceSensor;
    private boolean resetSlide = false;

    public void opModeMain() throws InterruptedException {
        this.initAll();
        distanceSensor = new TripWireDistanceSensor(hardwareMap, "distance_sensor", 8, () -> {
            this.leds.setPattern(BlinkinPattern.BLACK);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ignored) {
            }
            this.leds.setPattern(currentColor);
            return null;

        }, this::opModeIsActive, this::isStopRequested);
        distanceSensor.start();
        leds.setPattern(defaultColor);

        slide.autoMode();

        telemetry.addData("Initialization", "finished");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Declan's controls
            {
                driveTrain.gamepadControl(gamepad1, null);

                //Declan Speed Modifiers
                if (gamepad1.b) {
                    driveTrain.setDrivePowerMultiplier(0.6);
                }
                if (gamepad1.y) {
                    driveTrain.setDrivePowerMultiplier(1);

                }
                if (gamepad1.a) {
                    driveTrain.setDrivePowerMultiplier(0.3);
                }

            }

            //Eli's controls
            {
                //Handles Linear Slide Control
                {
                    if (!resetSlide) {
                        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                            manualSlideControl = true;
                            int pos = (int) (Math.abs(slide.getEncoderCount()) + (100 * -gamepad2.left_stick_y));
                            telemetry.addData("Pos", pos);
                            telemetry.update();
                            if (pos < -100) pos = -100;
                            if (pos > HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel))
                                pos = HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel);
                            slide.setTargetPosition(pos);
                        }

                        if (gamepad2.dpad_left) {
                            slide.setTargetLevel(HeightLevel.CappingUp);
                        }

                        if (gamepad2.dpad_right) {
                            slide.setTargetLevel(HeightLevel.CappingDown);
                        }

                        if (!gamepad2.dpad_up) {
                            dPadUpDepressed = true;
                        }

                        if (gamepad2.dpad_up && dPadUpDepressed) {
                            if (manualSlideControl) {
                                manualSlideControl = false;
                                currentLevel = HeightLevel.getClosestLevel(slide.getEncoderCount());
                            }
                            dPadUpDepressed = false;
                            currentLevel = currentLevel.add(1);
                            slide.setTargetLevel(currentLevel);

                        }

                        if (!gamepad2.dpad_down) {
                            dPadDownDepressed = true;
                        }

                        if (gamepad2.dpad_down && dPadDownDepressed) {
                            if (manualSlideControl) {
                                manualSlideControl = false;
                                currentLevel = HeightLevel.getClosestLevel(slide.getEncoderCount());
                            }
                            dPadDownDepressed = false;
                            currentLevel = currentLevel.subtract(1);
                            slide.setTargetLevel(currentLevel);
                        }

                        if (gamepad2.right_bumper && gamepad2.left_bumper) {
                            slide.teleopMode();
                            slideResetTimer.reset();
                            resetSlide = true;
                        }
                    } else if (slideResetTimer.seconds() > 0.5 && slideResetTimer.seconds() < 0.6) {
                        slide.setMotorPower(0.3);
                    } else if (slideResetTimer.seconds() > 1 && resetSlide) {
                        resetSlide = false;
                        slide.autoMode();
                        slide.resetEncoder();
                    }

                }

                //Intake Controls
                {
                    if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                        intake.setMotorPower(gamepad2.left_trigger - gamepad2.right_trigger);
                        BlinkinPattern o = outtake.getLEDPatternFromFreight();
                        if (o == null || !outtake.isClosed()) {
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
                }

                //Out take controls
                {
                    if (!gamepad2.y) {
                        y_depressed2 = true;
                    }
                    if (gamepad2.y && y_depressed2) {
                        y_depressed2 = false;
                        if (outtake.isClosed()) {
                            outtake.setServoOpen();
                            yTimer.reset();
                        } else {
                            outtake.setServoClosed();
                        }
                        leds.setPattern(defaultColor);
                        currentColor = defaultColor;
                    }

                    if (yTimer.seconds() > 1.25) {
                        outtake.setServoClosed();
                    }
                }


                //Carousel Spinner
                {
                    if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                        if (gamepad2.right_stick_x > 0) {
                            spinner.setPowerBlueDirection();
                        } else {
                            spinner.setPowerRedDirection();
                        }
                    } else {
                        spinner.stop();
                    }
                }

            }


        }
    }
}


