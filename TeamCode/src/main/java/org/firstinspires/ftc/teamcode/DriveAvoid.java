/*
// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of a MR
// gyro sensor along with a touch sensor. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses gyro to drive in a straight line when not avoiding an obstacle.
Reference: https://stemrobotics.cs.pdx.edu/node/4979
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Drive Avoid", group="Exercises")
//@Disabled
public class DriveAvoid extends LinearOpMode
{
    DcMotor                 leftMotor;
    DcMotor                 rightMotor;
    TouchSensor             touch;
    GyroSensor              gyro;
    double                  power = .30, correction;
    boolean                 aButton, bButton, touched;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to the touch sensor.
        touch = hardwareMap.touchSensor.get("touch_sensor");

        gyro = hardwareMap.gyroSensor.get("gyro");

        telemetry.addData("Mode", "starting gyro calibration...please wait");
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "gyro calibrated...waiting for start");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        gyro.resetZAxisIntegrator();

        while (opModeIsActive())
        {
            telemetry.addData("gyro heading", gyro.getHeading());
            telemetry.update();

            // Use gyro to drive in a straight line.
            correction = checkDirection();

            // set power levels.
            leftMotor.setPower(power - correction);
            rightMotor.setPower(power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            touched = touch.isPressed();

            if (touched || aButton || bButton)
            {
                // backup.
                leftMotor.setPower(power);
                rightMotor.setPower(power);

                sleep(500);

                // stop.
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                // turn 90 degrees right.
                if (touched || aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, heading, gain = .10;

        heading = gyro.getHeading();

        if (heading == 0)
            correction = 0;             // no adjustment.
        else if (heading > 180)
            correction = 360 - heading; // adjust left.
        else
            correction = -heading;      // adjust right.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 350 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        int     targetAngle;

        // reset gyro to zero.
        gyro.resetZAxisIntegrator();

        // Gyro returns 0->359 when rotating counter clockwise (left) and 359->0 when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
            targetAngle = 360 + degrees;    // degrees is - for right turn.
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
            targetAngle = degrees;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && gyro.getHeading() == 0)
            {
                telemetry.addData("gyro heading", gyro.getHeading());
                telemetry.update();
                idle();
            }

            while (opModeIsActive() && gyro.getHeading() > targetAngle)
            {
                telemetry.addData("gyro heading", gyro.getHeading());
                telemetry.update();
                idle();
            }
        }
        else
            while (opModeIsActive() && gyro.getHeading() < targetAngle)
            {
                telemetry.addData("gyro heading", gyro.getHeading());
                telemetry.update();
                idle();
            }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // Reset gyro heading to zero on new direction we are now pointing.
        gyro.resetZAxisIntegrator();
    }
}
