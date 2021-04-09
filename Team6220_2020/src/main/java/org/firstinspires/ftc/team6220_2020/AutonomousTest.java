package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.PIDFilter;

@Autonomous(name = "TestAutonomous", group = "Autonomous")
public class AutonomousTest extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

            driveInches(120, 90);
    }

    public void driveInches(double targetDistance, double degDriveAngle) {

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean targetReached = false;

        double xPosition = 0;
        double yPosition = 0;

        double radDriveAngle = Math.toRadians(degDriveAngle);
        double distanceLeft;

        PIDFilter translationPID;
        translationPID = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);

        while (!targetReached && opModeIsActive()) {
            // This calculates the distance traveled in inches
            double distanceTraveled = Math.sqrt(Math.pow((xPosition - 0), 2) + Math.pow((yPosition - 0), 2));

            // This adds a value to the PID loop so it can update
            distanceLeft = targetDistance - distanceTraveled;
            translationPID.roll(distanceLeft);

            // We drive the mecanum wheels with the PID value
            driveMecanum(radDriveAngle, Math.max(translationPID.getFilteredValue(), Constants.MINIMUM_DRIVE_POWER), 0.0);
            
            // Update positions using last distance measured by encoders
            xPosition = (Constants.IN_PER_ANDYMARK_TICK * (-motorFrontLeft.getCurrentPosition() +
                    motorBackLeft.getCurrentPosition() - motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4);

            yPosition = (Constants.IN_PER_ANDYMARK_TICK * (-motorFrontLeft.getCurrentPosition() -
                    motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4);

            telemetry.addData("X Position: ", xPosition);
            telemetry.addData("Y Position: ", yPosition);
            telemetry.addData("Distance Traveled: ", distanceTraveled);
            telemetry.update();

            if (distanceTraveled > targetDistance) {
                driveMecanum(radDriveAngle, 0.0, 0.0);
                targetReached = true;
            }
        }
    }
}