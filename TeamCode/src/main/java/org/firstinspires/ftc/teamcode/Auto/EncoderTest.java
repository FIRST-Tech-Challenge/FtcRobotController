package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class EncoderTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        // ------------------------------------------------------- //
        // Runs when the init button is pressed on driver hub

        telemetry.addData("Status", "Initializing");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized!");

        // ------------------------------------------------------- //
        waitForStart();

        while (opModeIsActive()) {
            // Runs in a loop when the start button is pressed on the driver hub
            moveMotors(1, 1, 1);

        }
    }

    public void moveMotors(int rightSide, int leftSide, float motorSpeed) {
        // Reset all encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor modes to run to position
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticksPerRevolution = 1440/60;
        float wheelCircumference = (float) 0.30787608005; // This is in meters
        int ticksPerMeter = (int) (ticksPerRevolution / wheelCircumference); //This should be ~4677 ticks

        int rightMoveDistance = ticksPerMeter * rightSide;   // Ex. 1 meter would be 4677 ticks
        int leftMoveDistance = ticksPerMeter * leftSide;     // Ex. 2 meters would be 9354 ticks

        frontRightMotor.setTargetPosition(rightMoveDistance);
        backRightMotor.setTargetPosition(rightMoveDistance);

        frontLeftMotor.setTargetPosition(leftMoveDistance);
        backLeftMotor.setTargetPosition(leftMoveDistance);

        frontLeftMotor.setPower(motorSpeed);
        frontRightMotor.setPower(motorSpeed);
        backLeftMotor.setPower(motorSpeed);
        backRightMotor.setPower(motorSpeed);
    }
}