package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterAutonomous extends MasterOpMode {

    double distanceTolerance = 0.5;
    double angleTolerance = 3;

    public void autoInitializeRobot()
    {
        super.initializeHardware();

        // zero the motor controllers before running; we don't know if motors start out at zero
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // todo Add turnSatisfactionCounter.
    // todo DON'T USE -180 WITH THIS METHOD, causes a wrap-around issue in the adjustAngles method
    // pivot using IMU, but with a reference start angle, but this angle has to be determined (read) before this method is called
    public void pivot(double targetAngle, double maxSpeed)
    {
        double pivotSpeed;
        double currentAngle;
        double errorAngle;
        int satisfactionCounter = 0;

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, stop motors

        do
        {
            robot.updatePosition();

            errorAngle = adjustAngles(robot.curAngle - targetAngle);
            turnFilter.roll(errorAngle);
            pivotSpeed = turnFilter.getFilteredValue();
            pivotSpeed = Range.clip(pivotSpeed, -maxSpeed, maxSpeed); // limit abs speed

            // add to angle satisfactionCounter if error is in certain range
            if (Math.abs(errorAngle) < 1) {
                satisfactionCounter++;
            }

            accelerationFilter.roll(pivotSpeed);
            if (Math.abs(robot.curAngle) < Math.abs(targetAngle) / 2) {
                pivotSpeed = accelerationFilter.getFilteredValue();
            }
            mecanumDrive(0, 0, pivotSpeed);

            telemetry.addData("CurAngle", robot.curAngle);
            telemetry.addData("ErrorAngle", errorAngle);
            telemetry.update();

            idle();

        } while (opModeIsActive() && (Math.abs(errorAngle) > angleTolerance)/* || (satisfactionCounter < 10)*/);

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    /*
           x and y are robot's current location
           targetx and target y is the location you want to go to
           current angle is usually the reference angle/ angle you are currently facing
           add a speed such that the robot does not overpower and stays in -1.0 and 1.0 range
        */
    // todo work on implementing movement using "driveMecanum" MasterOpMode
    public void move(double targetX, double targetY, double maxSpeed) throws InterruptedException {

        double movingPower;
        double turningPower;
        double errorX;
        double errorY;
        double distanceToTarget;
        double angleToTarget;
        double errorAngle;
        // getting initial distance
        double initialAngle = robot.curAngle;
        double initialDistanceToTarget = Math.hypot(targetX - robot.currentX, targetY - robot.currentY);

        //This clears any residual vales in our PID loop
        turnFilter.reset();
        moveFilter.reset();

        do {

            robot.updatePosition();

            // cos = adjacent/hypotenuse
            // math .cos returns in radians so convert it back to degrees
            // double errorX = Math.cos(errorAngle * Math.PI/180)  * distanceToTarget;
            //errorX = targetX - ( 0.5 * robot.currentX + 0.5 * robot.currentX);
            errorX = targetX - robot.currentX;

            // sin = opposite/ hypotenuse
            //double errorY = Math.sin(errorAngle * Math.PI/180) * distanceToTarget;
            //errorY = targetY - ( 0.5 * robot.currentX - 0.5 * robot.currentY);
            errorY = targetY - robot.currentY;


            // find distance to target with shortcut distance formula
            distanceToTarget = Math.hypot(errorX, errorY);
            // find angle using arc tangent 2 to preserve the sign and find angle to the target
            angleToTarget = Math.atan2(errorY, errorX);
            // adjust angle that you need to turn so it is not greater than 180
            errorAngle = adjustAngles(robot.curAngle - initialAngle);

            // scale vector
            // make sure the power is between 0-1 but maintaining x and y power ratios with the total magnitude
            moveFilter.roll(distanceToTarget);
            turnFilter.roll(errorAngle);

            // Get filtered movement and turning powers.  Also clip movement power to ensure we aren't moving too fast.
            movingPower = Range.clip(moveFilter.getFilteredValue(), -maxSpeed, maxSpeed);
            turningPower = turnFilter.getFilteredValue();

            // roll movingPower
            accelerationFilter.roll(movingPower);
            // limiting movement power acceleration if far away enough from target
            if (distanceToTarget > initialDistanceToTarget / 2 ) {
                movingPower = accelerationFilter.getFilteredValue();
            }
            mecanumDrive(angleToTarget, movingPower, turningPower);

            /*telemetry.addData("Loop Condition", (Math.abs(errorAngle) > angleTolerance || distanceToTarget > distanceTolerance));
            telemetry.addData("motorFL", motorFL.getCurrentPosition());
            telemetry.addData("motorBL", motorBL.getCurrentPosition());
            telemetry.addData("motorFR", motorFR.getCurrentPosition());
            telemetry.addData("motorBR", motorBR.getCurrentPosition());
            telemetry.addData("errorX", errorX);
            telemetry.addData("errorY", errorY);
            telemetry.addData("errorAngle", errorAngle);*/
            telemetry.addData("IMU Value", robot.getCorrectedHeading());
            telemetry.update();

            idle();
        } while ((Math.abs(errorAngle) > angleTolerance || distanceToTarget > distanceTolerance) && opModeIsActive());

        // stop motors
        telemetry.clear();
        //telemetry.addLine("stopping motors");
        telemetry.update();
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
