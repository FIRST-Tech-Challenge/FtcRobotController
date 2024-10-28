package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="AutoPinpoint")
//@Disabled

public class AutoOpMode extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine{
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3;
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 1000, -406, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,500,0, AngleUnit.DEGREES,0);

    final double SPEED = 0.3;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-101.6, -120.65); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        nav.initializeMotors();
        nav.setXYCoefficients(.005,0,2.0,DistanceUnit.MM,12);
        nav.setYawCoefficients(3.1,0,2.0, AngleUnit.DEGREES,2);

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
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            if(stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_1) {
                if (nav.driveTo(odo.getPosition(), TARGET_1, SPEED, 3)) {
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                }
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_2){
                if (nav.driveTo(odo.getPosition(), TARGET_2, SPEED, 3)) {
                    telemetry.addLine("at position #2!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;
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
            telemetry.addData("Position", data);
            telemetry.update();

        }
        while (opModeIsActive()) {
            // Print final pose
            pos = odo.getPosition();
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
        }
    }}
