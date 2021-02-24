package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class EncoderChassis extends BasicChassis {

    public EncoderChassis(LinearOpMode opMode) {
        super(opMode);
        op=opMode;
    }
    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public double[] track() {
        double[] data = {0, 0, 0};
        double diff[] = {motorLeftFront.getCurrentPosition() - encoder[0], motorRightFront.getCurrentPosition() - encoder[1], motorLeftBack.getCurrentPosition() - encoder[2], motorRightBack.getCurrentPosition() - encoder[3]};
        encoder[0] += diff[0];
        encoder[1] += diff[1];
        encoder[2] += diff[2];
        encoder[3] += diff[3];
        double x = (-(diff[0] + diff[3]) / (2 * counts_per_inch) + (diff[2] + diff[1]) / (2 * counts_per_inch));
        double y =  ((diff[0] + diff[3]) / (2 * counts_per_inch) + (diff[2] + diff[1]) / (2 * counts_per_inch));
        data[2]=getAngle();
        xpos += x*cos(data[2]*PI/180)+y*sin(data[2]*PI/180);
        ypos += y*cos(data[2]*PI/180)-x*sin(data[2]*PI/180);
        op.telemetry.addData("LeftFront", motorLeftFront.getCurrentPosition());
        op.telemetry.addData("RightFront", motorRightFront.getCurrentPosition());
        op.telemetry.addData("LeftBack", motorLeftBack.getCurrentPosition());
        op.telemetry.addData("RightBack", motorRightBack.getCurrentPosition());
        op.telemetry.addData("xpos", xpos);
        op.telemetry.addData("ypos", ypos);
        op.telemetry.addData("angle", getAngle());
        op.telemetry.update();
        data[0] = xpos;
        data[1] = ypos;
        data[2] = getAngle();
        return data;
    }
    public double getAngle() {
        double diff[] = {motorLeftFront.getCurrentPosition() - encoder[0], motorRightFront.getCurrentPosition() - encoder[1], motorLeftBack.getCurrentPosition() - encoder[2], motorRightBack.getCurrentPosition() - encoder[3]};
        encoder[0] += diff[0];
        encoder[1] += diff[1];
        encoder[2] += diff[2];
        encoder[3] += diff[3];
        double angle = ((encoder[0] + encoder[2]) - (encoder[1] + encoder[3])) / (counts_per_inch * 4 * robot_diameter * Math.PI * 360);
        angle %= 360;
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
    public void turnInPlace(double target, double power) {
        double currentAngle = getAngle();
        int direction = 1;
        double difference = currentAngle - target;
        double targetAngle = target;
        if (target < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.5)) {
            currentAngle = getAngle();
            difference = targetAngle - currentAngle;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.2 * power;
                }
            }
            motorRightBack.setPower(-power * direction);
            motorRightFront.setPower(-power * direction);
            motorLeftBack.setPower(power * direction);
            motorLeftFront.setPower(power * direction);
            op.telemetry.addData("current angle", currentAngle);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveForward(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        target_position[0] = currentPosition[0] + cos(getAngle() * PI / 180) * distance;
        target_position[1] = currentPosition[1] + sin(getAngle() * PI / 180) * distance;
        target_position[2] = currentPosition[2];
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

        op.telemetry.addData("ticks: ", (int) ticksToMove +
                "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
        op.telemetry.update();

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setPower(power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorLeftBack.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy())) {
            //correction = checkDirection();
            currentPosition = track();
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
            motorRightBack.setPower(power+anglecorrection);
            motorRightFront.setPower(power+anglecorrection);
            motorLeftBack.setPower(power-anglecorrection);
            motorLeftFront.setPower(power-anglecorrection);
//            op.telemetry.addData("correction", correction);
//            op.telemetry.update();
//            op.idle();
        }

        stopAllMotors();
        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveBackward(double distance, double power) {
        double ticksToMove = counts_per_inch * -distance;
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        target_position[0] = currentPosition[0] + cos(getAngle() * PI / 180) *- distance;
        target_position[1] = currentPosition[1] + sin(getAngle() * PI / 180) *- distance;
        target_position[2] = currentPosition[2];
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

        op.telemetry.addData("ticks: ", (int) ticksToMove +
                "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
        op.telemetry.update();

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setPower(power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorLeftBack.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy())) {
            //correction = checkDirection();
            currentPosition = track();
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
            motorRightBack.setPower(power+anglecorrection);
            motorRightFront.setPower(power+anglecorrection);
            motorLeftBack.setPower(power-anglecorrection);
            motorLeftFront.setPower(power-anglecorrection);
//            op.telemetry.addData("correction", correction);
//            op.telemetry.update();
//            op.idle();
        }

        stopAllMotors();
        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRight(double distance, double power) {//right
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() - ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() - ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        target_position[0] = currentPosition[0] + sin(getAngle() * PI / 180) * distance;
        target_position[1] = currentPosition[1] + cos(getAngle() * PI / 180) * distance;
        target_position[2] = currentPosition[2];
        motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("ticks: ", (int) ticksToMove +
                "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
        op.telemetry.update();

        motorLeftBack.setPower(power);
        motorRightBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightFront.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy())) {
            currentPosition = track();
            anglecorrection = (target_position[2]-currentPosition[2])*0.007;
            motorRightBack.setPower(power+anglecorrection);
            motorRightFront.setPower(power+anglecorrection);
            motorLeftBack.setPower(power-anglecorrection);
            motorLeftFront.setPower(power-anglecorrection);
        }

        stopAllMotors();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xpos += distance;
    }
    public void moveLeft(double distance, double power) {//left
        double ticksToMove = counts_per_inch * -distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() - ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() - ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        target_position[0] = currentPosition[0] + sin(getAngle() * PI / 180) * distance;
        target_position[1] = currentPosition[1] + cos(getAngle() * PI / 180) * distance;
        target_position[2] = currentPosition[2];
        motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("ticks: ", (int) ticksToMove +
                "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
        op.telemetry.update();

        motorLeftBack.setPower(power);
        motorRightBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightFront.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy())) {
            currentPosition = track();
            anglecorrection = (target_position[2]-currentPosition[2])*0.007;
            motorRightBack.setPower(power+anglecorrection);
            motorRightFront.setPower(power+anglecorrection);
            motorLeftBack.setPower(power-anglecorrection);
            motorLeftFront.setPower(power-anglecorrection);
        }
        stopAllMotors();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveAngle(double x, double y, double power) {
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startAngle = getAngle();
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection;
        target_position[0] = currentPosition[0] + x;
        target_position[1] = currentPosition[1] + y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(y, x) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        while (op.opModeIsActive() && (difference >= 1)) {
            currentPosition = track();
            /*op.telemetry.addData("targetx", target_position[0]);
            op.telemetry.addData("targety",target_position[1]);
            op.telemetry.addData("angle",angleInRadians);
            op.telemetry.addData("distance",difference);
            op.telemetry.addData("power1",anglePower[0]);
            op.telemetry.addData("power2",anglePower[1]);
            op.telemetry.update();
            op.telemetry.update();*/
            if (difference < 5) {
                power *= difference / 10;
                if (abs(power) < 0.2) {
                    power = 0.2;
                }
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(y, x) - currentPosition[2] * PI / 180;
            anglePower[0] = sin(angleInRadians + PI / 4);
            anglePower[1] = sin(angleInRadians - PI / 4);
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.007;
            if (difference > 10) {
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);

                }
            }
            motorRightBack.setPower(power * anglePower[1] + anglecorrection);
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
            difference = abs(sqrt((x) * (x) + (y) * (y)));
            op.telemetry.addData("distance", difference);
            op.telemetry.update();
        }
            turnInPlace(startAngle, 0.25);
            stopAllMotors();
        }

    @Override
    public void setPosition(float xPosition, float yPosition, float newangle) {

    }


    @Override
    public void goToPosition(double xPosition, double yPosition, double newangle, double power) {

    }

    @Override
    public void navigate() {

    }

    @Override
    public void navigateTeleOp() {

    }
}
