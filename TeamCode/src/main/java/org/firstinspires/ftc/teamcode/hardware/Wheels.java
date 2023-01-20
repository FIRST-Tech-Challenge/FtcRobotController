package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Wheels {

    public static final double TICKS_PER_ROTATION = 3611.2; // נמדד על ידי סיבוב הרובוט 20 פעם

    public static final double WHEEL_DIAMETER_CM = 9.6;
    public static final double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math.PI;
    public static final double MOTOR_ENCODER_RESOLUTION = 537.7;
    public static final double TICKS_PER_CM = MOTOR_ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE_CM;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final LinearOpMode opMode;

    public int weelsCurrentPosition = 0;

    public Wheels(LinearOpMode opMode) {
        this.opMode = opMode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "BackRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
        public void runWithEncoder() {
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public  void stop(){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public  void rotateByEncoder(double degrees, double power){
            double toPosition = degrees/360 * TICKS_PER_ROTATION;
            driveWheelToPosition(frontRight,power, -toPosition);
            driveWheelToPosition(frontLeft,power, toPosition);
            driveWheelToPosition(backRight, power, -toPosition);
            driveWheelToPosition(backLeft, power, toPosition);
            while (opMode.opModeIsActive() && motorsIsBussy()) {
                opMode.telemetry.addData("Front Position",  "%7d :%7d",
                        frontRight.getCurrentPosition(),
                        frontLeft.getCurrentPosition());
                opMode.telemetry.addData("Back Position",  "%7d :%7d",
                        backRight.getCurrentPosition(),
                        backLeft.getCurrentPosition());
                opMode.telemetry.update();
                opMode.idle();
            }
            stop();
        }
    public void driveForward(double distanceCM, double power){
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight,power, toPosition);
        driveWheelToPosition(frontLeft,power, toPosition);
        driveWheelToPosition(backRight, power, toPosition);
        driveWheelToPosition(backLeft, power, toPosition);
        while (opMode.opModeIsActive() && motorsIsBussy()) {
            opMode.telemetry.addData("Front Position",  "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position",  "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            opMode.idle();
        }
        stop();
    }
    public void driveBackrword(double distanceCM, double power){
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight,power, -toPosition);
        driveWheelToPosition(frontLeft,power, -toPosition);
        driveWheelToPosition(backRight, power, -toPosition);
        driveWheelToPosition(backLeft, power, -toPosition);
        while (opMode.opModeIsActive() && motorsIsBussy()) {
            opMode.telemetry.addData("Front Position",  "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position",  "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            opMode.idle();
        }
        stop();
    }

    public void driveLeft(double distanceCM, double power){
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight,power ,toPosition);
        driveWheelToPosition(frontLeft,power ,-toPosition);
        driveWheelToPosition(backRight,power ,-toPosition);
        driveWheelToPosition(backLeft,power ,toPosition);
        while (opMode.opModeIsActive() && motorsIsBussy()) {
            opMode.telemetry.addData("Front Position",  "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position",  "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            opMode.idle();
        }
        stop();
    }
    public void driveRight(double distanceCM, double power){
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight,power ,-toPosition);
        driveWheelToPosition(frontLeft,power ,toPosition);
        driveWheelToPosition(backRight,power ,toPosition);
        driveWheelToPosition(backLeft,power ,-toPosition);
        while (opMode.opModeIsActive() && motorsIsBussy()) {
            opMode.telemetry.addData("Front Position",  "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position",  "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            opMode.idle();
        }
        stop();
    }
    public boolean motorsIsBussy(){
        return  frontLeft  .isBusy() ||
                frontRight .isBusy() ||
                backLeft   .isBusy() ||
                backRight  .isBusy();
    }
    public void driveByEncoder(){
    }
    private void driveWheelToPosition(DcMotor wheel, double power, double toPosition){
        wheel.setTargetPosition((int) Math.round(toPosition) + wheel.getCurrentPosition());
        wheel.setPower(power);
        wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void driveWheelToAbsolutePosition(DcMotor wheel, double power, double toPosition){
        wheel.setTargetPosition((int) Math.round(toPosition));
        wheel.setPower(power);
        wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public double[] calculateMecanum(double x, double y, double rot){
        double fr = y - x - rot;
        double br = y + x - rot;
        double fl = y + x + rot;
        double bl = y - x + rot;
        opMode.telemetry.addData("Front Position",  "%7d :%7d",
                frontRight.getCurrentPosition(),
                frontLeft.getCurrentPosition());
        opMode.telemetry.addData("Back Position",  "%7d :%7d",
                backRight.getCurrentPosition(),
                backLeft.getCurrentPosition());
        return new double[]{fr,br,fl,bl};

    }
    public void driveByJoystick(double x, double y, double rot) {
        double[] wheelPower = calculateMecanum(x ,y ,rot);
        double fr = wheelPower[0];
        double br = wheelPower[1];
        double fl = wheelPower[2];
        double bl = wheelPower[3];

        double norm = max(max(abs(fr), abs(br)), max(abs(fr), abs(br)));

        if (norm > 1) {
            fr /= norm;
            br /= norm;
            fl /= norm;
            bl /= norm;
        }

        if (opMode.gamepad1.left_stick_button || opMode.gamepad1.left_trigger>.4) {
            fr *= .25;
            br *= .25;
            fl *= .25;
            bl *= .25;
        } else if (opMode.gamepad1.right_trigger>.4) {
            fr *= .8;
            br *= .8;
            fl *= .8;
            bl *= .8;
        } else {
            fr *= .6;
            br *= .6;
            fl *= .6;
            bl *= .6;
        }

        frontRight.setPower(fr);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
//        opMode.telemetry.addData("frontRight position: ", frontRight.getCurrentPosition());
//        opMode.telemetry.addData("frontLeft position: ", weel.getCurrentPosition());
//        opMode.telemetry.addData(weelName + " position: ", weel.getCurrentPosition());
//        opMode.telemetry.addData(weelName + " position: ", weel.getCurrentPosition());
    }
}
