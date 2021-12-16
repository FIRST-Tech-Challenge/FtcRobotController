package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static java.lang.Thread.sleep;

public class DriveMethod {
    public enum poseState {
        RED_WAREHOUSE,
        RED_STORAGEUNIT,
        BLUE_WAREHOUSE,
        BLUE_STORAGEUNIT,
        RED,
        BLUE,
        RED_OTHERS,
        BLUE_OTHERS,
        BLUE_SHARED_HUB,
        RED_SHARED_HUB,
        UNKNOWN
    }
    public enum alliance {
        RED,
        BLUE
    }
    public enum chassis {
        ONE,
        TWO
    }

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    public DriveMethod(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, DcMotor Intake, DcMotor Spin, DcMotor Slide, Servo Rotate, Servo Push, BNO055IMU imu){
        this.LF = LF;
        this.RF = RF;
        this.LB = LB;
        this.RB = RB;
        this.Intake = Intake;
        this.Spin = Spin;
        this.Slide = Slide;
        this.Rotate = Rotate;
        this.Push = Push;
        this.imu = imu;
    }

    public static poseState poseToState(Pose2d currentPose){
        double x = currentPose.getX();
        double y = currentPose.getY();
        if(x > 27.375 && y < -27.375){
            return poseState.RED_WAREHOUSE;
        }
        else if(x > 27.375 && y > 27.375){
            return poseState.BLUE_WAREHOUSE;
        }
        else if(y < 23.375 && y > -23.375 && x > 27.375){
            if(y > 0){
                return poseState.BLUE_SHARED_HUB;
            } else {
                return poseState.RED_SHARED_HUB;
            }
        }
        else if(x < -47.875 && y < 47.125 && y > 24.125){
            return poseState.BLUE_STORAGEUNIT;
        }
        else if(x < -47.875 && y > -47.125 && y < -24.125){
            return poseState.RED_STORAGEUNIT;
        }
        else if(y > 0){
            return poseState.BLUE_OTHERS;
        }
        else if(y < 0){
            return poseState.RED_OTHERS;
        }
        else{
            return poseState.UNKNOWN;
        }
    }

    public static double targetAngle(boolean isBlue, Pose2d pointA, Pose2d pointB){
        double p = pointA.getX();
        double q = pointA.getY();
        double m = pointB.getX();
        double n = pointB.getY();

        double slope = (q-n)/(p-m);
        double result = Math.abs(Math.atan(slope)+ Math.PI);

        if(isBlue) {
            //0~180
            if(result > Math.PI) {
                result = result - Math.PI;
            }
        }
        else {
            //180~360
            if(result < Math.PI) {
                result = result + Math.PI;
            }
        }
        return result;
    }

    public static Pose2d targetPlatePose(Pose2d currentPose, Pose2d targetPose, double radius){
        double p = targetPose.getX();
        double q = targetPose.getY();
        double m = currentPose.getX();
        double n = currentPose.getY();
        double r = radius;

        double a = 1 + Math.pow((q-n)/(p-m), 2.0);
        double b = 2*(((q-n)*(n*p-m*q)/Math.pow((p-m), 2.0))+(q*n-q*q)/(p-m)-p);
        double c = p*p+Math.pow((p*n-m*q)/(p-m), 2.0)-2.0*(p*n*q-m*q*q)/(p-m)+q*q-r*r;

        double x_1 = (-b + Math.sqrt(b*b-4.0*a*c))/2.0/a;
        double y_1 = (q-n)/(p-m)*x_1+(p*n-m*q)/(p-m);
        double x_2 = (-b - Math.sqrt(b*b-4.0*a*c))/2.0/a;
        double y_2 = (q-n)/(p-m)*x_2+(p*n-m*q)/(p-m);

        Pose2d targetPose_1 = new Pose2d(x_1, y_1);
        Pose2d targetPose_2 = new Pose2d(x_2, y_2);
        return nearestPoint(currentPose, targetPose_1, targetPose_2);
    }

    public static Pose2d nearestPoint(Pose2d currentPose, Pose2d targetPose_1, Pose2d targetPose_2){
        double x = currentPose.getX();
        double y = currentPose.getY();
        double x_1 = targetPose_1.getX();
        double y_1 = targetPose_1.getY();
        double x_2 = targetPose_2.getX();
        double y_2 = targetPose_2.getY();

        double distance_1 = Math.sqrt(Math.pow(Math.abs(x_1-x), 2.0) + (Math.pow(Math.abs(y_1-y), 2.0)));
        double distance_2 = Math.sqrt(Math.pow(Math.abs(x_2-x), 2.0) + (Math.pow(Math.abs(y_2-y), 2.0)));

        if(distance_1 > distance_2){
            return targetPose_2;
        }
        else{
            return targetPose_1;
        }
    }


    void twoPhaseSpin(boolean isReversed,double startingSpeed, double endSpeed) {
        double reverseFactor = 1;
        if(isReversed){
            reverseFactor = -1;
        }
        ElapsedTime tSpin = new ElapsedTime();
        double spinPower = startingSpeed * reverseFactor;
        while (tSpin.milliseconds() < 1000){
            Spin.setPower(spinPower);
        }
        while (tSpin.milliseconds() < 2500){
            spinPower = Range.clip(spinPower * tSpin.milliseconds()/1000.0, startingSpeed, endSpeed);
            Spin.setPower(spinPower);
        }
    }

    void stopMotion() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    double normalizeAngle(double angle) {
        double tempDeg = angle % 360;
        if (tempDeg >= 180) {
            tempDeg -= 360;
        } else if (tempDeg < -180) {
            tempDeg += 360;
        }
        return tempDeg;
    }

    double aquireHeading() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        sleep(20);
        return tempHead;
    }

//    void displayEncoderValue() {
//        try {
//            double LFDis = LF.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
//            double RFDis = RF.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
//            double LBDis = LB.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
//            double RBDis = RB.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
//            telemetry.addData("LF Encoder Value: ", LFDis);
//            telemetry.addData("RF Encoder Value: ", RFDis);
//            telemetry.addData("LB Encoder Value: ", LBDis);
//            telemetry.addData("RB Encoder Value: ", RBDis);
//            telemetry.addData("X Value: ", (LFDis / Math.sqrt(2.0) + RBDis / Math.sqrt(2.0) - LBDis / Math.sqrt(2.0) - RFDis / Math.sqrt(2.0)));
//            telemetry.addData("Y Value: ", (LFDis / Math.sqrt(2.0) + RFDis / Math.sqrt(2.0) + LBDis / Math.sqrt(2.0) + RBDis / Math.sqrt(2.0)));
//            telemetry.addData("Average Encoder Value: ", (LFDis + LBDis + RFDis + RBDis) / 4.0);
//            telemetry.update();
//        } catch (Exception e) {
//            telemetry.addLine("Unable to find encoder value");
//            telemetry.update();
//        }
//    }

    void driveStraight(boolean isForward, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
        }
        stopMotion();
    }



    void drivePerpendicularly(boolean isLeft, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int perpendicularFactor = -1;
        if (isLeft) {
            perpendicularFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = -1 * perpendicularFactor * power;
            LB_power = perpendicularFactor * power;
            RF_power = perpendicularFactor * power;
            RB_power = -1 * perpendicularFactor * power;
//            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
//                RF_power += perpendicularFactor * 0.1;
//                RB_power += perpendicularFactor * 0.1;
//                LF_power -= perpendicularFactor * 0.1;
//                LB_power -= perpendicularFactor * 0.1;
//            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
//                RF_power -= perpendicularFactor * 0.1;
//                RB_power -= perpendicularFactor * 0.1;
//                LF_power += perpendicularFactor * 0.1;
//                LB_power += perpendicularFactor * 0.1;
//            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
        }
        stopMotion();
    }

    void driveLeftTurn(boolean isForward, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(-RF_power);
            LB.setPower(LB_power);
            RB.setPower(-RB_power);
//            telemetry.addData("RF_power", -RF_power);
//            telemetry.addData("RB_power", -RB_power);
//            telemetry.addData("LF_power", LF_power);
//            telemetry.addData("LB_power", LB_power);
//
//            telemetry.update();
//            displayEncoderValue();
        }
        stopMotion();
    }

    void driveRightTurn(boolean isForward, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {

            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;

            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(-LF_power);
            RF.setPower(RF_power);
            LB.setPower(-LB_power);
            RB.setPower(RB_power);
//            telemetry.addData("RF_power", RF_power);
//            telemetry.addData("RB_power", RB_power);
//            telemetry.addData("LF_power", -LF_power);
//            telemetry.addData("LB_power", -LB_power);
//
//            telemetry.update();
//            displayEncoderValue();
        }
        stopMotion();
    }
    void driveStrafeLeft(boolean isForward, double margin, double power, double timeInterval) throws InterruptedException {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {

            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;

            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(0);
            RF.setPower(RF_power);
            LB.setPower(-LB_power);
            RB.setPower(0);
//            telemetry.addData("RF_power", RF_power);
//            telemetry.addData("RB_power", RB_power);
//            telemetry.addData("LF_power", -LF_power);
//            telemetry.addData("LB_power", -LB_power);
//            telemetry.update();
//            displayEncoderValue();
        }
        stopMotion();
    }

    //make a turn that based on the current heading in a certain direction and angle
    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) throws InterruptedException {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) throws InterruptedException {
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if (currentAngle - targetAngle > 0) {
            if (currentAngle - targetAngle < 180) {
                //cw
                angleFactor = -1;
            } else {
                //ccw
                angleFactor = 1;
            }
        } else {
            if (targetAngle - currentAngle < 180) {
                //ccw
                angleFactor = 1;
            } else {
                //cw
                angleFactor = -1;
            }
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin) && (tempAngle > targetAngle - margin))) {
            tempAngle = aquireHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
//            telemetry.addData("RF_power", RF_power);
//            telemetry.addData("RB_power", RB_power);
//            telemetry.addData("LF_power", LF_power);
//            telemetry.addData("LB_power", LB_power);
//            telemetry.update();
        }
        stopMotion();
    }

    boolean slideMotion(int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
//        telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
//        telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
//        telemetry.update();
        return Slide.isBusy();
    }

    boolean rotateMotion(boolean moveOut) throws InterruptedException {
        if(moveOut)
            Rotate.setPosition(1.0);
        else
            Rotate.setPosition(0.03);
//        telemetry.addLine("Rotate: " + Rotate.getPosition());
//        telemetry.update();
        sleep(300);
        return (Rotate.getPosition() == 1.0 || Rotate.getPosition() == 0.03);
    }

    boolean pushMotion(boolean moveOut) throws InterruptedException {
        if(moveOut)
            Push.setPosition(0.4);
        else
            Push.setPosition(0.0);
//        telemetry.addLine("Push: " + Push.getPosition());
//        telemetry.update();
        sleep(300);
        return (Push.getPosition() == 0.0 || Push.getPosition() == 0.4);
    }

    boolean spinMotion(double power, double timeInterval) {
        ElapsedTime spinTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (spinTime.milliseconds() <= timeInterval) {
            Spin.setPower(power);
        }
        Spin.setPower(0);
        return Spin.isBusy();
    }
}
