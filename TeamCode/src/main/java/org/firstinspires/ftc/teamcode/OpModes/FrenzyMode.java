package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

@TeleOp(name = "Frenzy", group = "Robot15173")
public class FrenzyMode extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();
    IBaseOdometry odometry = null;

    // Timing related variables
    ElapsedTime runtime = new ElapsedTime();
    boolean buttonPressable = true;
    double DEBOUNCE_DELAY_TIME_MS = 200;
    double lastButtonPressed = 0;

    // Intake related variables
    boolean changedIntake = false;
    boolean intakeReverse = false;

    // Rotator related variable
    boolean changedRotator = false;

    @Override
    public void runOpMode() {
        try {
            try{
                robot.init(this, this.hardwareMap, telemetry);
                odometry =  VSlamOdometry.getInstance(this.hardwareMap, 20);
                odometry.setInitPosition(50, 15, 0); // TODO: Remove this
                Thread odometryThread = new Thread(odometry);
                odometryThread.start();
            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

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
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

                handleSpecialActions();

                telemetry.addData("Left front", robot.getLeftOdometer());
                telemetry.addData("Right front", robot.getRightOdometer());
                telemetry.addData("X", odometry.getCurrentX());
                telemetry.addData("Y", odometry.getCurrentY());
                telemetry.addData("Heading", odometry.getCurrentHeading());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            robot.activateIntake(0);
            robot.activateRotator(0);
            robot.activateLift(0);
            if (odometry != null) {
                odometry.stop();
            }
        }
    }

    protected void handleSpecialActions() {
        handleLift();

        // BUTTON PRESSABLE
        buttonPressable = ((runtime.milliseconds() - lastButtonPressed) >= DEBOUNCE_DELAY_TIME_MS);

        if (buttonPressable) {
            handleIntake();
            handleRotator();
        }
    }

    protected void handleIntake() {
        // MOVE INTAKE
        if (gamepad1.a) {
            recordButtonPressed();
            changedIntake = !changedIntake;
        }

        if (changedIntake) {
            robot.activateIntake(0.95);
        } else {
            robot.activateIntake(0);
        }

        if (gamepad1.b){
            recordButtonPressed();
            intakeReverse = !intakeReverse;
        }

        if (intakeReverse){
            robot.activateIntake(-0.75);
        } else {
            robot.activateIntake(0);
        }
    }

    protected void handleRotator() {
        if (gamepad1.y) {
            recordButtonPressed();
            changedRotator = !changedRotator;
        }

        if (changedIntake) {
            robot.activateRotator(0.5);
        } else {
            robot.activateRotator(0);
        }
    }

    protected void handleLift() {
        if (gamepad1.dpad_up) {
            robot.activateLift(0.5);
        }
        else if (gamepad1.dpad_down) {
            robot.activateLift(-0.5);
        }
        else {
            robot.activateLift(0);
        }
    }

    private void recordButtonPressed() {
        lastButtonPressed = runtime.milliseconds();
    }
}
