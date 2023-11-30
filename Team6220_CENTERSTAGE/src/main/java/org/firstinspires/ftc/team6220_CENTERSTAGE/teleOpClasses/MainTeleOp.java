package org.firstinspires.ftc.team6220_CENTERSTAGE.teleOpClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.firstinspires.ftc.team6220_CENTERSTAGE.MecanumDrive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Utilities;

@TeleOp(name="League2_TeleOp", group ="vazkiiIsANeatByMod")
public class MainTeleOp extends LinearOpMode {

    // define states for turning, using slides, outtaking

    // stop posting about ENUMS im TIRED of SEEING IT
    // my friends on discord send me ENUMS
    // in person its hecking ENUMS
    // i was in this server right?
    // and ALLLL the channels were just "STATE MACHINE"
    // ENUMS
    // GRAAAAHH

    private int slidePreset = 0;
    private int intakePreset = 0;

    TurnStates curTurningState = TurnStates.TURNING_MANUAL;
    enum TurnStates {
        TURNING_MANUAL,
        TURNING_90,
        TURNING_FIELD_CENTRIC
    }

    SlideStates curSlideState = SlideStates.SLIDES_MANUAL;
    enum SlideStates {
        SLIDES_MANUAL,
        SLIDES_FULL_EXTEND,
        SLIDES_FULL_RETRACT
    }

    OuttakeStates curOuttakeState = OuttakeStates.OUTTAKE_REFILL;
    enum OuttakeStates {
        OUTTAKE_REFILL,
        OUTTAKE_CLOSED,
        OUTTAKE_DROP_1,
        OUTTAKE_DROP_2
    }

    // toggle for tilting outtake forward and back
    boolean outtakeTiltedForward = false;

    // drive powers, read from input and then manipulated every loop
    double drivePowerX = 0.0;
    double drivePowerY = 0.0;
    double turnPower = 0.0;
    double intakePower = 0.0;

    // holds heading from imu read which is done in roadrunner's mecanum drive class for us
    double currentHeading = 0.0;
    double targetHeading = 0.0;

    // useful groups of keycodes
    final GamepadKeys.Button[] BUMPER_KEYCODES = {
            GamepadKeys.Button.LEFT_BUMPER,
            GamepadKeys.Button.RIGHT_BUMPER
    };
    final GamepadKeys.Button[] DPAD_KEYCODES = {
            GamepadKeys.Button.DPAD_UP,
            GamepadKeys.Button.DPAD_LEFT,
            GamepadKeys.Button.DPAD_DOWN,
            GamepadKeys.Button.DPAD_RIGHT
    };

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            // update gamepads and read inputs
            gp1.readButtons();
            gp2.readButtons();

            drivePowerX = gp1.getLeftX() * Constants.DRIVE_POWER_X_MULTIPLIER;
            drivePowerY = gp1.getLeftY() * Constants.DRIVE_POWER_Y_MULTIPLIER;
            turnPower = gp1.getRightX();

            intakePower = Math.max(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                                   gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
                                * Constants.INTAKE_POWER_MULTIPLIER;
            

            // get heading from imu in degrees
            currentHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            // update turn state
            if (Math.abs(turnPower) > Constants.TURN_STICK_DEADZONE) {
                targetHeading = 0.0;
                curTurningState = TurnStates.TURNING_MANUAL;

            } else if (Utilities.justPressedAny(gp1, BUMPER_KEYCODES) > -1) {
                // sets target to 90 added to current heading
                // if already turning 90, add to target instead of current
                // also limits target angle between -180 and 180
                targetHeading = Utilities.limitAngle(
                    (curTurningState == TurnStates.TURNING_90 ? targetHeading : currentHeading) +
                    (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ? 90 : -90)
                );
                curTurningState = TurnStates.TURNING_90;

            } else if (Utilities.justPressedAny(gp1, DPAD_KEYCODES) > -1) {
                // find which dpad button was pressed, use its index * 90 degrees as target
                // up: 0*90=0, left: 1*90=90, down: 2*90=180, right: 3*90=270 (-> -90)
                targetHeading = Utilities.limitAngle(Utilities.justPressedAny(gp1, DPAD_KEYCODES) * 90.0);
                curTurningState = TurnStates.TURNING_FIELD_CENTRIC;

            } else if (Math.abs(Utilities.shortestDifference(currentHeading, targetHeading)) < Constants.TELEOP_MIN_HEADING_ACCURACY) {
                curTurningState = TurnStates.TURNING_MANUAL;
            }

            // use turn state
            switch (curTurningState) {
                case TURNING_MANUAL:
                    turnPower *= Constants.TURN_POWER_MULTIPLIER;
                    break;
                case TURNING_90:
                    // falls through to field centric
                case TURNING_FIELD_CENTRIC:
                    // https://www.desmos.com/calculator/zdsmmtbnwf (updated)
                    // flips difference so that it mimics turn stick directions
                    turnPower = Utilities.clamp(-Utilities.shortestDifference(currentHeading, targetHeading) / 90.0);
            }

            // check for slowmode
            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0 ||
                    gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0) {

                drivePowerX *= Constants.SLOWMODE_MULTIPLIER;
                drivePowerY *= Constants.SLOWMODE_MULTIPLIER;
                turnPower *= Constants.SLOWMODE_MULTIPLIER;
            }
            // clamp powers between -1.0 and 1.0
            drivePowerX = Utilities.clamp(drivePowerX);
            drivePowerY = Utilities.clamp(drivePowerY);
            turnPower = Utilities.clamp(turnPower);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            drivePowerY, // not negative so it goes the right direction
                            -drivePowerX
                    ),
                    -turnPower
            ));

            // NOTE /!\ this doesn't work yet as we haven't set it up
            drive.updatePoseEstimate();


            if (!drive.isDevBot) {

                // drive Drone Launcher 📄📄✈✈✈
                if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    drive.droneServo.setPosition(Constants.DRONE_SERVO_LAUNCHING_POS);
                } else if (gp2.wasJustReleased(GamepadKeys.Button.X)){
                    drive.droneServo.setPosition(Constants.DRONE_SERVO_PRIMED_POS);
                }

                // Slides stuff 😎📈📈📈
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
                    if (slidePreset > 0) {
                        slidePreset--;
                    }

                // slides stuff 2: the greg
                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
                    if (slidePreset > Constants.SLIDE_POSITIONS.length) {
                        slidePreset++;
                    }

                // moves the slides to a preset (theoretically)
                moveSlides(Constants.SLIDE_POSITIONS[slidePreset]);

                // move intake bar down to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                    if (intakePreset > 0) {
                        intakePreset--;
                    }

                // move intake bar up to preset value with dpad but only if it can
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                    if (intakePreset < Constants.INTAKE_POSITIONS.length - 1) {
                        intakePreset++;
                    }

                // drive the servo to position set by dpad using above code
                drive.intakeServo.setPosition(Constants.INTAKE_POSITIONS[intakePreset]);

                // go go gadget intake motor
                drive.intakeMotor.setPower(-gp2.getLeftY());
            }


            // telemetry
            telemetry.addData("current turning state", curTurningState);
            telemetry.addData("imu reading", currentHeading);
            telemetry.addData("target heading", targetHeading);
            telemetry.addData("turn power", turnPower);
            telemetry.addData("drive power X", drivePowerX);
            telemetry.addData("drive power Y", drivePowerY);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("intake preset", intakePreset);

            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading", drive.pose.heading);

            telemetry.update();
        }
    }

    /**
     * Calculates error then sets the slide powers to drive the slides to the target.
     *
     * Note: Does not actually go until the slides hit the target, this method is meant to be
     * used in conjunction with a outside loop ♻
     * @param targetPos the position to target in encoder ticks
     */
    private void moveSlides(int targetPos) {
        double power = (targetPos - drive.slideMotor.getCurrentPosition()) * Constants.SLIDE_P_GAIN;
        drive.slideMotor.setPower(power);
        drive.returnMotor.setPower(power * Constants.SLIDE_RETURN_MOTOR_POWER_MUL + Constants.SLIDE_RETURN_MOTOR_POWER_OFFSET);
    }
}
