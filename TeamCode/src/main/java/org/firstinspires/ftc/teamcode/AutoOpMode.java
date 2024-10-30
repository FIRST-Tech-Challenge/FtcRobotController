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
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3;
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 2000, -806, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,0,-1200, AngleUnit.DEGREES,0);

    boolean armMoving = false;

    final double SPEED = 0.3;

    final double ARM_POWER = -0.3;
    final int ARM_UP_MAX_POSITION = -2700;
    final int ARM_DOWN_POSITION = 0;
    final int EXT_MAX_POSITION = 10000;
    final int EXT_IN_POSITION = 0;


    @Override
    public void runOpMode() {
        Timer t;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        initPinpoint();

        nav.initializeMotors();
        nav.setXYCoefficients(.005,0,2.0,DistanceUnit.MM,12);
        nav.setYawCoefficients(3.1,0,2.0, AngleUnit.DEGREES,2);

        rotateArmMotor = hardwareMap.get(DcMotor.class, "rotate");
        rotateArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rotateArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArmMotor = hardwareMap.get(DcMotor.class, "extender");
        //extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

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
            odo.update();

//            while (rotateArmMotor.getCurrentPosition() > ARM_UP_MAX_POSITION) {
//                rotateArmMotor.setPower(0.5);
//            }

            if(stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_1) {
               if (nav.driveTo(odo.getPosition(), TARGET_1, SPEED, 3)) {
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
               }
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_2) {
                if (rotateArmMotor.getCurrentPosition() > ARM_UP_MAX_POSITION) {
                    rotateArmMotor.setPower(ARM_POWER);
                }
                else {
                    rotateArmMotor.setPower(0);
                }
                if (nav.driveTo(odo.getPosition(), TARGET_2, SPEED, 3) &&
                    rotateArmMotor.getCurrentPosition() <= ARM_UP_MAX_POSITION) {
                    telemetry.addLine("**************************************");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                }
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_3){
                if (nav.driveTo(odo.getPosition(), TARGET_3, SPEED, 3)){
                    telemetry.addLine("at position #3!");
                    stateMachine = StateMachine.AT_TARGET;
                }
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


    private void armUp() {
        if (!armMoving) {
//            rotateArmMotor.setTargetPosition(ARM_UP_MAX_POSITION);
//            rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateArmMotor.setPower(ARM_POWER);
            armMoving = true;
        }
    }

    private void armDown() {
        rotateArmMotor.setTargetPosition(ARM_DOWN_POSITION);
        rotateArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotateArmMotor.setPower(-ARM_POWER);
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
