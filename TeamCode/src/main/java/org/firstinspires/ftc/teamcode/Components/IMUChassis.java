/**
 * Warren, i appoint you the owner of this file.
 * Please clean up
 *
 * @author: Warren
 * @version: 1.0
 * @status: work in progress
 */

package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class IMUChassis extends BasicChassis {

    /* local OpMode members. */

    private BNO055IMU imu;
    private Orientation             lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;
    private double IMUgain = -0.15;

    double[] encoder = new double[4];
    double xpos = 0;
    double ypos = 0;

    //set true to enable imu vice versa
    final boolean enableIMU = true;

    public IMUChassis(LinearOpMode opMode) {
        super(opMode);
        op = opMode;

        lastAngles  = new Orientation();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // make sure the imu gyro is calibrated before continuing.
        while (!op.isStopRequested() && !imu.isGyroCalibrated())
        {
            op.sleep(50);
            op.idle();
        }

        op.telemetry.addData("Mode", "waiting for start");
        op.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        op.telemetry.update();
        op.sleep(500);

        // Chassis Motors
    }

    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //op.telemetry.addData("first angle: ", (int)angles.firstAngle);
        //op.telemetry.update();
        //op.sleep(1000);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle <= -180) //If the angle is -180, it should be 180, because they are at the same point. The acceptable angles are (-180, 180]
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .1;

        angle = getAngle();

        /*if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;*/

        if (enableIMU == false) {
            correction = 0;
        } else {
            correction = angle * (-gain);
        }
        return correction;
    }

    /****Moving Robot****/

    @Override
    public void turnInPlace(double target, double power) {
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle = getAngle();
        double newTarget = target;
        double error = target-currentAngle;
        double gain = -0.005;
        double leftPower = power*gain*error;
        double rightPower = -leftPower; // power*gain*error

        if (leftPower>1) {leftPower=1;}
        if (rightPower>1) {rightPower=1;}
        if (leftPower<-1) {leftPower=-1;}
        if (rightPower<-1) {rightPower=-1;}

        if (newTarget>180){newTarget=newTarget-360;}
        if (newTarget<=-180){newTarget=newTarget+360;}

        motorLeftBack.setPower(leftPower);
        motorLeftFront.setPower(leftPower);
        motorRightBack.setPower(rightPower);
        motorRightFront.setPower(rightPower);

        while (op.opModeIsActive() && (error > 2 || error < -2))
        {
            currentAngle = getAngle();
            error = target - currentAngle;
            leftPower = power*gain*error*4;
            rightPower = -power*gain*error*4;
            //op.telemetry.addData("TurnIMU", "Angle"+currentAngle+ "Error"+ error+"LP"+leftPower+ "RP"+ rightPower);
            //op.telemetry.update();
            if (leftPower>1) {leftPower=1;}
            if (rightPower>1) {rightPower=1;}
            if (leftPower<-1) {leftPower=-1;}
            if (rightPower<-1) {rightPower=-1;}
            if(abs(leftPower)<0.1){
                leftPower*=2;
            }
            if(abs(rightPower)<0.1){
                rightPower*=2;
            }
            motorLeftBack.setPower(leftPower);
            motorLeftFront.setPower(leftPower);
            motorRightBack.setPower(rightPower);
            motorRightFront.setPower(rightPower);
            op.idle();
        }

        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightBack.setPower(0);

    }

    @Override
    public void moveForward(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double startingAngle = 0;

        startingAngle = getAngle();

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = newLeftBackTargetPosition - currentPosition; //interchangable based on which way robot turns

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = newLeftBackTargetPosition - currentPosition;
            currentAngle = getAngle();
            correction = (currentAngle - startingAngle) * .01;//gain
            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(power + correction);
            motorLeftBack.setPower(power - correction);
            motorLeftFront.setPower(power + correction);
            op.telemetry.addData("current pos", currentPosition + "delta pos", deltaPosition);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    @Override
    public void moveBackward(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() - ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double startingAngle = 0;

        startingAngle = getAngle();

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = newLeftBackTargetPosition - currentPosition; //interchangable based on which way robot turns

        while (op.opModeIsActive() && (deltaPosition <= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = newLeftBackTargetPosition - currentPosition;
            currentAngle = getAngle();
            //correction = (currentAngle - startingAngle) * .01;//gain
            motorRightBack.setPower(-power - correction);
            motorRightFront.setPower(-power + correction);
            motorLeftBack.setPower(-power - correction);
            motorLeftFront.setPower(-power + correction);
            op.telemetry.addData("current pos", currentPosition + "delta pos", deltaPosition);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    @Override
    public void moveRight(double distance, double power) {
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double maxcorrection = power;

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = ticksToMove;

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = newLeftBackTargetPosition - currentPosition;
            currentAngle = getAngle();
            correction = (currentAngle - getAngle()) * IMUgain * power;//gain is 0.06 for power 0.85
            if (correction > maxcorrection) {
                correction = maxcorrection;
            }
            if (correction < -maxcorrection) {
                correction = -maxcorrection;
            }
            motorRightBack.setPower(-power - correction);
            motorRightFront.setPower(power - correction);
            motorLeftBack.setPower(power + correction);
            motorLeftFront.setPower(-power + correction);
        }

        stopAllMotors();
    }

    @Override
    public void moveLeft(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() - ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double maxcorrection = power;

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = ticksToMove;

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = currentPosition - newLeftBackTargetPosition;
            currentAngle = getAngle();
            correction = (currentAngle - getAngle()) * IMUgain * power;//gain=0.06 for power 0.85
            if (correction > maxcorrection) {
                correction = maxcorrection;
            }
            if (correction < -maxcorrection) {
                correction = -maxcorrection;
            }
            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(-power - correction);
            motorLeftBack.setPower(-power + correction);
            motorLeftFront.setPower(power + correction);
        }
        stopAllMotors();
    }

    public double[] track() {
            double[] data = {0, 0, 0};
            double diff[] = {motorLeftFront.getCurrentPosition() - encoder[0], motorRightFront.getCurrentPosition() - encoder[1], motorLeftBack.getCurrentPosition() - encoder[2], motorRightBack.getCurrentPosition() - encoder[3]};
            encoder[0] += diff[0];
            encoder[1] += diff[1];
            encoder[2] += diff[2];
            encoder[3] += diff[3];
            double x = (-(diff[0] + diff[3]) / (4* counts_per_inch) + (diff[2] + diff[1]) / (4 * counts_per_inch));
            double y =  ((diff[0] + diff[3]) / (4 * counts_per_inch) + (diff[2] + diff[1]) / (4 * counts_per_inch));
            data[2]=getAngle();
            xpos += x*cos(data[2]*PI/180)+y*sin(data[2]*PI/180);
            ypos += y*cos(data[2]*PI/180)-x*sin(data[2]*PI/180);
        /*op.telemetry.addData("LeftFront", motorLeftFront.getCurrentPosition());
        op.telemetry.addData("RightFront", motorRightFront.getCurrentPosition());
        op.telemetry.addData("LeftBack", motorLeftBack.getCurrentPosition());
        op.telemetry.addData("RightBack", motorRightBack.getCurrentPosition());
        op.telemetry.addData("xpos", xpos);
        op.telemetry.addData("ypos", ypos);
        op.telemetry.addData("angle", getAngle());
        op.telemetry.update();*/
            data[0] = xpos;
            data[1] = ypos;
            data[2] = getAngle();
            return data;
    }

    @Override
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
        angleInRadians = atan2(y, x);
        anglePower[0] = sin(angleInRadians + PI / 4);
        anglePower[1] = sin(angleInRadians - PI / 4);
        anglecorrection = (currentPosition[2] - target_position[2]) * 0.025;
        if (difference > 10) {
            if (abs(anglePower[1]) > abs(anglePower[0])) {
                anglePower[1] *= abs(1 / anglePower[1]);
                anglePower[0] *= abs(1 / anglePower[1]);
            } else {
                anglePower[1] *= abs(1 / anglePower[0]);
                anglePower[0] *= abs(1 / anglePower[0]);

            }
        }
        motorRightBack.setPower(power * anglePower[1] - anglecorrection);
        motorRightFront.setPower(power * anglePower[0] - anglecorrection);
        motorLeftBack.setPower(power * anglePower[0] + anglecorrection);
        motorLeftFront.setPower(power * anglePower[1] + anglecorrection);
        difference = abs(sqrt((x) * (x) + (y) * (y)));
        op.telemetry.addData("distance", difference);
        op.telemetry.update();
    }
    stopAllMotors();
}

    @Override
    public void setPosition(double xPosition, double yPosition, double newangle) {

    }

    @Override
    public void goToPosition(double xPosition, double yPosition, double newangle, double power) {

    }
}
