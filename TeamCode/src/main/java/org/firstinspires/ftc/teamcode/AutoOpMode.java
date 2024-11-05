package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



import java.util.Locale;
import java.util.Timer;

@Autonomous(name="AutoPinpoint")
//@Disabled

public class AutoOpMode extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private final DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
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

    // Field poses
    static final Pose2D SUBMERSIBLE_POS = new Pose2D(DistanceUnit.MM, 500, -400, AngleUnit.DEGREES, 0);
    static final Pose2D OBSERVATION_ZONE = new Pose2D(DistanceUnit.MM,0,-600, AngleUnit.DEGREES,0);

    final double DRIVE_SPEED = 0.3;
    final double ARM_POWER = -0.2;
    final double EXTEND_POWER = 0.3;
    final int ARM_UP_MAX_POSITION = -2000;
    final int ARM_DOWN_POSITION = 0;
    final int EXT_MAX_POSITION = 2000;
    final int EXT_PLACE_POSITION = 900;


    @Override
    public void runOpMode() {
        Timer t;

        // Initialize the Pinpoint
        initPinpoint();

        // Initiaize DriveToPoint
        nav.initializeMotors();
        nav.setXYCoefficients(.01,0,2.0,DistanceUnit.MM,12);
        nav.setYawCoefficients(3.1,0,2.0, AngleUnit.DEGREES,2);

        // Initialize motors and servos
        rotateArmMotor = hardwareMap.get(DcMotor.class, "rotate");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor = hardwareMap.get(DcMotor.class, "extender");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Print current state
        telemetry.addData("State: ", stateMachine);
        telemetry.addData("Pose X(mm): ", odo.getPosition().getX(DistanceUnit.MM));
        telemetry.addData("Pose Y(mm): ", odo.getPosition().getY(DistanceUnit.MM));
        telemetry.addData("Pose Heading(deg): ", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("ArmPosition", rotateArmMotor.getCurrentPosition());
        telemetry.addData("ExtensionPosition", extendArmMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        //telemetry.setAutoClear(false);

        while (opModeIsActive()) {
            telemetry.addData("State: ", stateMachine);
            telemetry.addData("Pose X(mm): ", odo.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("Pose Y(mm): ", odo.getPosition().getY(DistanceUnit.MM));
            telemetry.addData("Pose Heading(deg): ", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.addData("ArmPosition", rotateArmMotor.getCurrentPosition());
            telemetry.addData("ExtensionPosition", extendArmMotor.getCurrentPosition());

            // Pinpoint update must be called every cycle
            odo.update();

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
                boolean cond1 = nav.driveTo(odo.getPosition(), SUBMERSIBLE_POS, DRIVE_SPEED, 0);
                boolean cond2 = armUp(ARM_UP_MAX_POSITION);
                boolean cond3 = armExtend(EXT_MAX_POSITION);

                //if all 3 conditions met, move to next state
                if (cond1 && cond2 && cond3) {
                        stateMachine = StateMachine.PLACE_SPECIMEN;
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
            if (stateMachine == StateMachine.DRIVE_TO_OBSERVATION_ZONE) {
                // Drive to observation zone
                boolean cond = nav.driveTo(odo.getPosition(), OBSERVATION_ZONE, DRIVE_SPEED, 0);
                if (cond) {
                    stateMachine = StateMachine.END;
                }
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

    private void initPinpoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-101.6, -120.65); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                 GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }
}
