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
    boolean changedclaw = true;
    boolean changedintake = false;
    boolean changedshooter = false;
    boolean intakeReverse = false;
    boolean buttonpressable = true;
    boolean shooterslower = false;
    boolean changedguard = false;
    double delaytime = 200;
    double startdelay = 0;
    double grabdelay = 0;
    private BotThreadAction bta = null;
    private BotThreadAction turretbta = null;
    Thread btaThread = null;
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
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

            locator = new RobotCoordinatePosition(robot, RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();

            // init turret
            robot.initTurretThread(this);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            robot.activatTurretThread(this);
            turretbta = new BotThreadAction(robot, telemetry, "moveTurretCams", this);
            Thread moveCamTurretThread = new Thread(turretbta);
            moveCamTurretThread.start();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
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

                buttonpressable = ((runtime.milliseconds() - startdelay) >= delaytime);

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

                // move claw
                if (gamepad1.dpad_right && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedclaw = !changedclaw;
                }

                if (changedclaw) {
                    robot.closeWobbleClaw();
                } else {
                    robot.openWobbleClaw();
                }


                // move swing thread
//                if (gamepad1.dpad_up) {
//                    bta = new BotThreadAction(robot, telemetry, "wobbleforward", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.dpad_down) {
//                    bta = new BotThreadAction(robot, telemetry, "wobbleback", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.dpad_left) {
//                    bta = new BotThreadAction(robot, telemetry, "wobblewall", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.x && buttonpressable) {
//                    startdelay = runtime.milliseconds();
//                    bta = new BotThreadAction(robot, telemetry, "wallclose", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                    changedclaw = !changedclaw;
//                }


                // wobble swing regular
                if (gamepad1.dpad_up && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.forwardWobbleSwing();
                } else if (gamepad1.dpad_down && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.backWobbleSwing();
                } else if (gamepad1.dpad_left && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedguard = !changedguard;
                } else if (gamepad1.x && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.liftWallGrab();
                    changedclaw = !changedclaw;
                }

//                if (gamepad1.y && buttonpressable) {
//                    startdelay = runtime.milliseconds();
//                    shooterslower = !shooterslower;
//                }


                if (changedguard) {
                    robot.guardDown();
                } else {
                    robot.guardUp();
                }

                // move intake
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

                // move shooter
                if (gamepad1.left_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedshooter = !changedshooter;
                }

                if (gamepad2.left_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedshooter = !changedshooter;
                }

                if (changedshooter) {
                    if (shooterslower) {
                        robot.shootermed();
                    } else {
                        robot.shooter();
                    }
                } else {
                    robot.stopshooter();
                }

                // shoot with servo
                if (gamepad1.right_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.shootServo();
                }

                runShootPegSequence();

                telemetry.addData("X ", locator.getXInches() );
                telemetry.addData("Y ", locator.getYInches() );
                telemetry.addData("Orientation (Degrees)", locator.getOrientation());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(10000);
        }
        finally {
            robot.stopintake();
            robot.stopshooter();
            robot.stopTurretAngler();
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
//            robot.shootPegSequence(locator);
            robot.shootPegSequenceManual(locator);
//            robot.shootPegContinuous(locator);
            gamepadRateLimit.reset();
        }
    }
}
