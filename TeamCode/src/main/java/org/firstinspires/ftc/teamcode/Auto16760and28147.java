package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;
import java.util.Timer;

@Autonomous(name="Auto16760and28147")
//@Disabled

public class Auto16760and28147 extends LinearOpMode {

    private DcMotor rotateArmMotor = null;
    private DcMotor extendArmMotor = null;
    private final ElapsedTime timer = new ElapsedTime();

    // Auto State Machine
    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        PLACE_SPECIMEN,
        RELEASE_SPECIMEN,
        DRIVE_TO_OBSERVATION_ZONE,
        END
    }

    final double DRIVE_SPEED = 0.3;
    final double ARM_POWER = -1;
    final double EXTEND_POWER = 1;
    final int ARM_UP_MAX_POSITION = -4000;
    final int ARM_DOWN_POSITION = 0;
    final int EXT_MAX_POSITION = 10000;
    final int EXT_PLACE_POSITION = 5000;

    boolean firstTime = true;

    @Override
    public void runOpMode() {
        Timer t;

        // Initialize Drive subsystem
        Drive drive = new Drive(hardwareMap, telemetry);

        // Initialize motors and servos
        rotateArmMotor = hardwareMap.get(DcMotor.class, "Elevation");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor = hardwareMap.get(DcMotor.class, "Extension");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            //----------------------------------------------------------
            // State: WAITING_FOR_START
            //----------------------------------------------------------
            if (stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_SUBMERSIBLE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_SUBMERSIBLE
            // Actions: rotate arm
            //          extend arm
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE) {
                // In parallel:
                // (a) drive to front of SUBMERSIBLE
                // (b) rotate arm to ARM_UP_MAX_POSITION
                // (c) extend arm to EXT_MAX_POSITION
                // When all three conditions met, move to next state (PLACE_SPECIMEN)
                if (firstTime) {
                    drive.setTargetEncoderValue(48);
                    firstTime = false;
                }
                boolean cond1 = drive.moveRobot(Drive.Direction.Forward, DRIVE_SPEED);
                boolean cond2 = armUp(ARM_UP_MAX_POSITION);
                boolean cond3 = armExtend(EXT_MAX_POSITION);

                //if all 3 conditions met, move to next state
                if (cond1 && cond2 && cond3) {
                    stateMachine = StateMachine.PLACE_SPECIMEN;
                    sleep(2000);
                }
            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            //----------------------------------------------------------
            if (stateMachine == StateMachine.PLACE_SPECIMEN){
                //boolean cond = armExtend(EXT_MAX_POSITION);
                boolean cond = armRetract(EXT_PLACE_POSITION);
                if (cond) {
                    stateMachine = StateMachine.RELEASE_SPECIMEN;
                }
            }

            //----------------------------------------------------------
            // State: RELEASE_SPECIMEN
            // Actions: open fingers to release specimen
            //----------------------------------------------------------
            if (stateMachine == StateMachine.RELEASE_SPECIMEN){
                // Release specimen
                // TODO
                stateMachine = StateMachine.DRIVE_TO_OBSERVATION_ZONE;
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: drive to observation zone and park
            //----------------------------------------------------------

            /*
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                // Drive to observation zone
                boolean cond = nav.driveTo(odo.getPosition(), OBSERVATION_ZONE, DRIVE_SPEED, 0);
                if (cond) {
                    stateMachine = StateMachine.END;
                }
            }
            */


            //----------------------------------------------------------
            // State: END
            // Actions: Done with auto routine
            //----------------------------------------------------------
            if (stateMachine == StateMachine.END) {
                sleep(5000);
                break;
            }

            telemetry.update();
        }
    }

    // Move arm up until the newPosition reached
    private boolean armUp(int newPosition) {
        boolean done = false;
        if (rotateArmMotor.getCurrentPosition() > newPosition) {
            // raise arm
            rotateArmMotor.setPower(ARM_POWER);
        }
        else { // arm is <= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Move arm down until newPosition reached
    private boolean armDown(int newPosition) {
        boolean done = false;
        if (rotateArmMotor.getCurrentPosition() < newPosition) {
            // lower arm
            rotateArmMotor.setPower(-ARM_POWER);
        }
        else { //arm is >= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armExtend(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() < newPosition) {
            // extend arm
            extendArmMotor.setPower(EXTEND_POWER);
        }
        else { //extension is >= newPosition so stop motor
            extendArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armRetract(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() > newPosition) {
            // retract arm
            extendArmMotor.setPower(-EXTEND_POWER);
        }
        else { //extension is <= newPosition so stop motor
            extendArmMotor.setPower(0);
            done = true;
        }
        return done;
    }
}
