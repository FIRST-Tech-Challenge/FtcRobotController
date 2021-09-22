package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.mason;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;


@TeleOp(name = "Beeline TeleOp Test", group = "Linear Opmode")
public class Beeline_TeleOp extends LinearOpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;
    double LMultiplier;
    double RMultiplier;
    double originalAngle;

    public Beeline_TeleOp() {
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left  = hardwareMap.get(DcMotor.class, "leftMotors");
        right = hardwareMap.get(DcMotor.class, "rightMotors");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        double LPower;
        double RPower;
        LMultiplier = 0.95;
        RMultiplier = 1;

        waitForStart();

        boolean releasedRightBumper = true;
        boolean releasedLeftBumper = true;
        boolean releasedX = true;
        boolean releasedA = true;
        boolean releasedY = true;
        boolean toggleX = true;
        boolean toggleY = true;



        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            if(gamepad1.right_bumper) {
                if(releasedRightBumper && releasedLeftBumper) {
                    increaseSpeed(0.05);
                    releasedRightBumper = false;
                }
            } else if(!releasedRightBumper){
                releasedRightBumper = true;
            }

            if(gamepad1.left_bumper){
                if(releasedRightBumper && releasedLeftBumper) {
                    decreaseSpeed(0.05);
                    releasedLeftBumper = false;
                }
            } else if (!releasedLeftBumper){
                releasedLeftBumper = true;
            }

            if(gamepad1.a){
                if(releasedA) {
                    decreaseSpeed(speed / 2.0);
                    releasedA = false;
                }

            } else if(!releasedA){
                increaseSpeed(speed);
                releasedA = true;
            }

            if (gamepad1.y) {
                if (releasedY){
                    if (toggleY) {
                        toggleY = false;
                    } else {
                        toggleY = true;
                    }
                    releasedY = false;
                }
            } else if (!releasedY){
                releasedY = true;
            }


            if (gamepad1.x) {
                if (releasedX){
                    if (toggleX) {
                        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        toggleX = false;
                    } else {
                        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                        toggleX = true;
                    }
                    releasedX = false;
                }
            } else if (!releasedX){
                releasedX = true;
            }


            LPower  = Range.clip(LMultiplier*(gamepad1.left_trigger + speed*(drive + rotate)), -1.0, 1.0) ;
            RPower  = Range.clip(RMultiplier*(gamepad1.right_trigger + speed*(drive - rotate)), -1.0, 1.0) ;


            left.setPower(LPower);
            right.setPower(RPower);


            telemetry.addData("Left (%.2f)", LPower);
            telemetry.addData("Right (%.2f)", RPower);
            //telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, rotate);
            telemetry.addData("Speed (%.2f)", speed);
            if(!toggleX){
                telemetry.addLine("BREAK");
            } else{
                telemetry.addLine("FLOAT");
            }
            if(toggleY){
                if(drive == 0 && rotate == 0)
                    originalAngle = getHeading();
                telemetry.addData("Heading (%.2f)", originalAngle);
            }

            telemetry.update();
        }
    }

    private void decreaseSpeed(double s) {
        double decreased = speed - s;
        if (decreased < 0) {
            speed = 0;
            return;
        }
        speed = decreased;
    }

    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
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

    double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        telemetry.addData("Heading", tempHead);
        telemetry.update();
        sleep(20);
        return tempHead;
    }

    //make a turn that based on the current heading in a certain direction and angle
    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = getHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) {
        int angleFactor = 0;
        final double currentAngle = getHeading();
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
            tempAngle = getHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);

            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
    }
}