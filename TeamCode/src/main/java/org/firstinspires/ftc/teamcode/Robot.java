package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Lint;

public class Robot {

    // Define Motors and sensors
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    DcMotor[] driveMotors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};

    public OpMode systemTools;


    //Define other variables
    HardwareMap hwMap;

    //Constructor Method
    public Robot(HardwareMap hwMapIn, OpMode systemToolsIn) {
        hwMap = hwMapIn;
        systemTools = systemToolsIn;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_drive");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_drive");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        stopAllMotors();
        startDriveEncoders();

        // Define and Initialize Sensors


    }

    //Autonomous Movement Commands (forward, strafe, turn, ect.)
    public void GoDistanceCM(int centimeters, double power, LinearOpMode linearOpMode) {
        // holds the conversion factor for TICKS to centimeters
        final double conversionFactor = 21.4; // Will need to change upon robot testing

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversionFactor);

        resetDriveEncoders();

        //Calculate the target for each specific motor
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        setDriveTarget(FLtarget, FRtarget, BLtarget, BRtarget);

        startDriveEncodersTarget();

        setDrivePower(power, power, power, power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        while (linearOpMode.opModeIsActive() &&
                //     (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        stopDriveMotors();
    }

    // Basic movement commands (powers, drivemodes)
    public void setDrivePower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void setDriveTarget(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        frontLeftMotor.setTargetPosition(frontLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backLeftMotor.setTargetPosition(backLeftTarget);
        backRightMotor.setTargetPosition(backRightTarget);
    }

    public void stopDriveMotors() {
        setDrivePower(0,0,0,0);
    }

    public void stopAllMotors() {
        stopDriveMotors();
        // Add any other motors we use here and set them to 0 power
    }

    public void resetDriveEncoders() {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void startDriveEncoders() {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void startDriveEncodersTarget() {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode, boolean TelemetryOn) {

        final double conversionFactor = 8.46; // for outreach robot: 8.46, for FreightFrenzy robot: TBD

        if (degrees < 0 && power > 0) power = -power;

        int ticks = (int) Math.abs(Math.round(degrees * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }

        int FLtarget = frontLeftMotor.getCurrentPosition() + ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() - ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        while (linearOpMode.opModeIsActive() &&
              (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(frontRightMotor.getCurrentPosition()) < ticks && Math.abs(backLeftMotor.getCurrentPosition()) < ticks && Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();
        }

    }

    public void StrafeCM(int Centimeters, double Power, LinearOpMode linearOpMode, boolean TelemetryOn) {

        final double conversionFactor = 8.46; // outreach robot: 8.46 FreightFrenzy robot: TBD
        if (Centimeters < 0 && Power > 0) Power = -Power;

        int ticks = (int) Math.abs(Math.round(Centimeters * conversionFactor));

        if (TelemetryOn) {
            systemTools.telemetry.addData("Status", "Resetting Encoders");
            systemTools.telemetry.update();
        }

        resetDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition());
            systemTools.telemetry.update();
        }

        int FLtarget = frontLeftMotor.getCurrentPosition() - ticks;
        int FRtarget = frontRightMotor.getCurrentPosition() + ticks;
        int BLtarget = backLeftMotor.getCurrentPosition() + ticks;
        int BRtarget = backRightMotor.getCurrentPosition() - ticks;

        startDriveEncoders();

        frontLeftMotor.setPower(-Power);
        frontRightMotor.setPower(Power);
        backRightMotor.setPower(Power);
        backLeftMotor.setPower(-Power);

        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(frontRightMotor.getCurrentPosition()) < ticks && Math.abs(backLeftMotor.getCurrentPosition()) < ticks && Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        if (TelemetryOn) {
            systemTools.telemetry.addData("Path", "Complete");
            systemTools.telemetry.addData("counts", ticks);
            systemTools.telemetry.update();

        }

    }
}
