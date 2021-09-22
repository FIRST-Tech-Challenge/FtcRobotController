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

@TeleOp(name = "Mecanum Augmented TeleOp Test", group = "Linear Opmode")
public class MecanumAugmented_TeleOp extends LinearOpMode{
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        telemetry.addData("Gyro Calibration Status", imu.getCalibrationStatus().toString());

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;
        double XPower;
        double YPower;

        waitForStart();

        boolean releasedRightBumper = true;
        boolean releasedLeftBumper = true;
        boolean releasedGamePad1 = true;
        boolean releasedA = true;

        boolean toggleGamePad1 = true;

        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
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


            if (gamepad1.x) {
                if (releasedGamePad1){
                    if (toggleGamePad1) {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleGamePad1 = false;
                    } else {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleGamePad1 = true;
                    }
                    releasedGamePad1 = false;
                }
            } else if (!releasedGamePad1){
                releasedGamePad1 = true;
            }

            LFPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate - strafe), -1.0, 1.0) ;


            Double currentSpeed[] = {LFPower, LBPower, RFPower, RBPower};
            speedList.add(currentSpeed);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            /*if(loop == 200){
                if (reverse){
                    reverse = false;
                }
                telemetry.addData("duration of a loop: %.2f", runtime.milliseconds());
                sleep(100);
                loop = 0;
            }*/
            telemetry.update();
            //loop++;

//            released = true;
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

    double aquireHeading() {
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
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) {
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
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
    }

}

