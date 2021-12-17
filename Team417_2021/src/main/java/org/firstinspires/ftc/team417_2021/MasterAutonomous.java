package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.util.Range;

abstract public class MasterAutonomous extends MasterOpMode {
    double angleTolerance = 1;
    double targetAngle = 0;
    public void pivot (double targetAngle, double maxSpeed) {
        double pivotSpeed;
        double errorAngle;
        this.targetAngle = targetAngle;

        do {
            robot.updatePosition();
            errorAngle = adjustAngle(robot.getCorrectedHeading() - targetAngle);

            turnFilter.roll(errorAngle);
            pivotSpeed = turnFilter.getFilteredValue();

            if (Math.abs(pivotSpeed) > maxSpeed){
                pivotSpeed = Math.signum(pivotSpeed) * maxSpeed;
            }

            accelerationFilter.roll(pivotSpeed);
            if (Math.abs(robot.curAngle) < Math.abs(targetAngle) / 2) {
                pivotSpeed = accelerationFilter.getFilteredValue();
            }

            drive(0, pivotSpeed);

            telemetry.addData("CurAngle", robot.curAngle);
            telemetry.addData("ErrorAngle", errorAngle);
            telemetry.update();

        } while (opModeIsActive() && (Math.abs(errorAngle) > angleTolerance));

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void moveInches(double inches, double maxSpeed) throws InterruptedException {
        double movingPower;
        double turningPower;
        double targetAngle = robot.getCorrectedHeading();
        double errorAngle;
        double initialInches = robotInches();
        double errorDistance;
        double currentInches;
        double distanceTolerance = 0.5;

        turnFilter.reset();
        moveFilter.reset();
        do {
            currentInches = robotInches() - initialInches;
            errorDistance = inches - currentInches;
            errorAngle = robot.getCorrectedHeading() - targetAngle;

            moveFilter.roll(errorDistance);
            turnFilter.roll(errorAngle);

            movingPower = Range.clip(moveFilter.getFilteredValue(), -maxSpeed, maxSpeed);
            turningPower = turnFilter.getFilteredValue();
            drive(movingPower, turningPower);
            telemetry.addData("error", errorAngle);
            telemetry.addData("heading", robot.getCorrectedHeading());
            telemetry.addData("dist", errorDistance);
            telemetry.update();
        }
        while ((Math.abs(errorAngle) > 3 || Math.abs(errorDistance) > distanceTolerance) && opModeIsActive());
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        telemetry.clear();
    }

    // returns average value of all drive encoders, converted to inches
    public double robotInches() {
        return ((float) motorFL.getCurrentPosition() + motorBL.getCurrentPosition() +
                motorFR.getCurrentPosition() + motorBR.getCurrentPosition() ) / ( 4 * COUNTS_PER_INCH );
    }

    public void move(double targetX, double targetY, double maxSpeed) throws InterruptedException {
        double movingPower;
        double turningPower;
        double totalPower;
        double movingRatio;
        double turningRatio;

        double errorX;
        double errorY;
        double angleToTarget;
        double errorAngle;
        double initialDistanceToTarget = Math.hypot(targetX - robot.currentX, targetY - robot.currentY);
        double distanceToTarget;
        double distanceTolerance = 0.5;

        //This clears any residual vales in our PID loop
        turnFilter.reset();
        moveFilter.reset();

        do {
            robot.updatePosition();
            errorX = robot.currentX - targetX;
            errorY = robot.currentY - targetY;
            angleToTarget = Math.atan2(errorY, errorX);
            errorAngle = robot.getCorrectedHeading() - angleToTarget;
            distanceToTarget = Math.hypot(errorX, errorY);

            moveFilter.roll(distanceToTarget);
            turnFilter.roll(errorAngle);

            movingPower = Range.clip(moveFilter.getFilteredValue(), -maxSpeed, maxSpeed);
            turningPower = turnFilter.getFilteredValue();
            totalPower = movingPower + turningPower;
            totalPower = Range.clip(totalPower, - maxSpeed, maxSpeed);

            movingRatio = movingPower / (totalPower);
            turningRatio = turningPower / (totalPower);

            accelerationFilter.roll(totalPower);

            if (distanceToTarget > initialDistanceToTarget / 2 ) {
                movingPower = accelerationFilter.getFilteredValue() * movingRatio;
                turningPower = accelerationFilter.getFilteredValue() * turningRatio;
            }
            drive(movingPower, turningPower);

            telemetry.addData("Current X", robot.currentX);
            telemetry.addData("Current Y", robot.currentY);
            telemetry.addData("Angle to target", angleToTarget);
            telemetry.addData("Moving Power", movingPower);
            telemetry.addData("Turning Power", turningPower);
            telemetry.addData("Current Heading", robot.curAngle);
            telemetry.update();

            idle();
        } while ((Math.abs(errorAngle) > angleTolerance || distanceToTarget > distanceTolerance) && opModeIsActive());

        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
        telemetry.clear();
    }

    public double adjustAngle(double angle){
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
