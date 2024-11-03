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
    private DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
    private DcMotor rotateArmMotor = null;
    private DcMotor extendArmMotor = null;
    private ElapsedTime timer = new ElapsedTime();

    enum StateMachine{
        WAITING_FOR_START,
        DRIVE_TO_SUBMERSIBLE,
        PLACE_SPECIMEN
        RELEASE_SPECIMEN,
        DRIVE_TO_OBSERVATION_ZONE;
    }

    static final Pose2D SUBMERSIBLE_POS = new Pose2D(DistanceUnit.MM, 2000, -806, AngleUnit.DEGREES, 0);
    static final Pose2D OBSERVATION_ZONE = new Pose2D(DistanceUnit.MM,0,-1200, AngleUnit.DEGREES,0);

    boolean armMoving = false;

    final double SPEED = 0.3;
    final double ARM_POWER = -0.3;
    final double EXTEND_POWER = 0.3;
    final int ARM_UP_MAX_POSITION = -2700;
    final int ARM_DOWN_POSITION = 0;
    final int EXT_MAX_POSITION = 10000;
    final int EXT_PLACE_POSITION = 9000;


    @Override
    public void runOpMode() {
        Timer t;

        // Initialize the Pinpoint
        initPinpoint();

        // Initiaize DriveToPoint
        nav.initializeMotors();
        nav.setXYCoefficients(.005,0,2.0,DistanceUnit.MM,12);
        nav.setYawCoefficients(3.1,0,2.0, AngleUnit.DEGREES,2);

        // Initialize motors and servos
        rotateArmMotor = hardwareMap.get(DcMotor.class, "rotate");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArmMotor = hardwareMap.get(DcMotor.class, "extender");
        extendArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Output current status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("ArmPosition", rotateArmMotor.getCurrentPosition());
        //telemetry.addData("ExtensionPosition", armExtensionMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

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
                if (nav.driveTo(odo.getPosition(), SUBMERSIBLE_POS, SPEED, 3) &&
                    armUp(ARM_UP_MAX_POSITION) &&
                    armExtend(EXTEND_MAX_POSITION)) {
                    stateMachine = StateMachine.PLACE_SPECIMEN;
                }
            }

            //----------------------------------------------------------
            // State: PLACE_SPECIMEN
            // Actions: retract arm until target location reached
            //----------------------------------------------------------
            if (stateMachine == StateMachine.PLACE_SPECIMEN){
                if (armRetract(EXT_PLACE_POSITION)){
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
            }

            //----------------------------------------------------------
            // State: DRIVE_TO_OBSERVATION_ZONE
            // Actions: drive to observation zone and park
            //----------------------------------------------------------
            if (stateMachine == StateMachine.DRIVE_TO_SUBMERSIBLE){
                // Drive to observation zone
                if (nav.driveTo(odo.getPosition(), OBSERVATION_ZONE, SPEED, 3) {
                    stateMachine = StateMachine.END;
            }

            telemetry.addData("current state:",stateMachine);
            pos = odo.getPosition();
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pose", data);
            telemetry.addData("ArmPosition", rotateArmMotor.getCurrentPosition());
            telemetry.addData("ArmPower", rotateArmMotor.getPower());
            //telemetry.addData("ExtendPosition", extendArmMotor.getCurrentPosition());
            //telemetry.addData("ExtendPower", extendArmMotor.getPower());
            telemetry.update();

        }
        while (opModeIsActive()) {
            telemetry.addData("current state:",stateMachine);
            pos = odo.getPosition();
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pose", data);
            telemetry.addData("ArmPosition", rotateArmMotor.getCurrentPosition());
            telemetry.addData("ArmPower", rotateArmMotor.getPower());
            //telemetry.addData("ExtensionPosition", armExtensionMotor.getCurrentPosition());
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
        else { arm is <= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Move arm down until newPosition reached
    private void armDown(int newPosition) {
        boolean done = false;
        if (rotateArmMotor.getCurrentPosition() < newPosition) {
            // lower arm
            rotateArmMotor.setPower(-ARM_POWER);
        }
        else { arm is >= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armExtend(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() < newPosition) {
            // raise arm
            extendArmMotor.setPower(EXTEND_POWER);
        }
        else { extension is >= newPosition so stop motor
            rotateArmMotor.setPower(0);
            done = true;
        }
        return done;
    }

    // Extend arm out until the newPosition reached
    private boolean armRetract(int newPosition) {
        boolean done = false;
        if (extendArmMotor.getCurrentPosition() > newPosition) {
            // raise arm
            extendArmMotor.setPower(-EXTEND_POWER);
        }
        else { extension is <= newPosition so stop motor
            rotateArmMotor.setPower(0);
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
