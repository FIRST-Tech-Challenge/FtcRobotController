package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.DummyBot;
import org.firstinspires.ftc.teamcode.bots.SwingPosition;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.BotThreadAction;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

import java.util.concurrent.TimeUnit;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

// Main Op Mode
@TeleOp(name = "Ultimate", group = "Robot15173")
public class UltimateMode extends LinearOpMode {

    // Declare OpMode members.
    UltimateBot robot = new UltimateBot();
    private ElapsedTime runtime = new ElapsedTime();

    //Just a comment to see if I can push code to github-Rudra
    // Declare Toggle Booleans
    boolean changedclaw = true;
    boolean changedintake = false;
    boolean changedshooter = false;
    boolean intakeReverse = false;
    boolean changedguard = false;
//    boolean autoShooter = true;

    // Declare Button Delays
    boolean buttonpressable = true;
    double delaytime = 200;
    double startdelay = 0;

    // Run Bot Thread Action
    private BotThreadAction bta = null;
//    private BotThreadAction turretbta = null;
    Thread btaThread = null;

    // Locator Variables
    RobotCoordinatePosition locator = null;
    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.setDriveToPowerMode();
            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();

            // locator thread initialization
            locator = new RobotCoordinatePosition(robot, RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
            Thread positionThread = new Thread(locator);
            positionThread.start();

            // init turret
//            robot.initTurretThread(this);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // start turret tracking
//            robot.activatTurretThread(this);
//            turretbta = new BotThreadAction(robot, telemetry, "moveTurretCams", this);
//            Thread moveCamTurretThread = new Thread(turretbta);
//            moveCamTurretThread.start();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // DRIVING
                double drive = gamepad1.left_stick_y;
                double turn = 0;
                double ltrigger = gamepad1.left_trigger;
                double rtrigger = gamepad1.right_trigger;
                if (ltrigger > 0) {
                    turn = -ltrigger;
                } else if (rtrigger > 0) {
                    turn = rtrigger;
                }

                double strafe = gamepad1.right_stick_x;

                if (Math.abs(strafe) > 0) {
                    telemetry.addData("Strafing", "Left: %2f", strafe);
                    telemetry.update();
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

                // BUTTON PRESSABLE
                buttonpressable = ((runtime.milliseconds() - startdelay) >= delaytime);

                // MOVE CLAW
                if (gamepad1.dpad_right && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedclaw = !changedclaw;
                }

                if (changedclaw) {
                    robot.closeWobbleClaw();
                } else {
                    robot.openWobbleClaw();
                }


                // MOVE WOBBLE THREAD
                if (gamepad1.dpad_up) {
                    bta = new BotThreadAction(robot, telemetry, "wobbleforward", this);
                    btaThread = new Thread(bta);
                    btaThread.start();
                } else if (gamepad1.dpad_down) {
                    bta = new BotThreadAction(robot, telemetry, "wobbleback", this);
                    btaThread = new Thread(bta);
                    btaThread.start();
                } else if (gamepad1.x && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    bta = new BotThreadAction(robot, telemetry, "wallclose", this);
                    btaThread = new Thread(bta);
                    btaThread.start();
                    changedclaw = !changedclaw;
                }


                // CHANGE GUARD POSITION
                if (gamepad1.dpad_left && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedguard = !changedguard;
                }

                if (changedguard) {
                    robot.guardDown();
                } else {
                    robot.guardUp();
                }

                // MOVE INTAKE
                if (gamepad1.a && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedintake = !changedintake;
                }

                if (changedintake) {
                    robot.intake();
                } else {
                    robot.stopintake();
                }

                if (gamepad1.b && buttonpressable){
                    startdelay = runtime.milliseconds();
                    intakeReverse = !intakeReverse;
                }

                if (intakeReverse){
                    robot.intakeReverse();
                }

                // MOVE SHOOTER
                if (gamepad1.left_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedshooter = !changedshooter;
                }

                if (changedshooter) {
                    robot.shooter();
                } else {
                    robot.stopshooter();
                }

                // SHOOT SERVO
                if (gamepad1.right_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.shootServo();
                }
                if (gamepad2.right_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.shootServo();
                }

                // TOGGLE AUTO TURRET
//                if (gamepad2.x && buttonpressable) {
//                    startdelay = runtime.milliseconds();
//                    autoShooter = !autoShooter;
//                    robot.changeTurretSync();
//                }

                // ROTATE TURRET MANUALLY
                if (gamepad2.dpad_left && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.turretLittleLeft();
                } else if (gamepad2.dpad_right && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.turretLittleRight();
                }

                // SHOOT PEGS
                runShootPegSequence();

                telemetry.addData("X ", locator.getXInches() );
                telemetry.addData("Y ", locator.getYInches() );
                telemetry.addData("Orientation (Degrees)", locator.getOrientation());
//                telemetry.addData("Turret Detect X", robot.getRawXDetect());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            robot.stopintake();
            robot.stopshooter();
//            robot.stopTurretAngler();
            if (locator != null){
                locator.stop();
            }
        }
    }

    private void runShootPegSequence(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }
        if (gamepad1.y){
            robot.shootPegTurnManual(locator);
//            robot.shootPegTurn(locator);
//            robot.shootPegTurnManualTape(locator);
            gamepadRateLimit.reset();
        }
    }
}
