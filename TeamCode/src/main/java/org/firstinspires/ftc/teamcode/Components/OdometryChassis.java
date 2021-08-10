package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

//2.0,1.7,1.1
public class OdometryChassis extends BasicChassis {
    VuforiaWebcam vuforia = null;
    static DcMotorEx odom1;
    static DcMotorEx odom2;
    static DcMotorEx odom3;
    double xVelocity = 0;
    double yVelocity = 0;
    double aVelocity = 0;
    double Velocity=0;
    public static final boolean gotoPosition_off = false;
    public static final boolean vuforia_on = false;
    final int[] odomconst = {1, -1, -1};
    final float ticks_per_inch = (float) (8640 * 2.54 / 38 * Math.PI) * 72 / 76;
    float robot_diameter = (float) sqrt(619.84);
    static final float[] odom = {0f,0f,0f};
    private LinearOpMode op = null;
    private final BNO055IMU imu;
    private Orientation lastAngles = null;
    public static float globalAngle = 0;
    public static float xpos = 0;
    public static float ypos = 0;
    public static float angle;
    final double[] velocity = {0,0,0};
    double power = .30, correction;


    //set true to enable imu vice versa
    final boolean isCorgi=true;
    final ElapsedTime runtime = new ElapsedTime();
    double lastTime = 0;
    double thisTime = 0;

    public OdometryChassis(LinearOpMode opMode, boolean navigator, boolean tobeCorgiornottobeCorgi) {
        super(opMode);
        op = opMode;
        xpos = 0;
        ypos = 0;
        angle = 0;
        // Chassis encoders
        odom1 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        odom3 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        odom2 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
        // reset encoder count.
        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lastAngles = new Orientation();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // make sure the imu gyro is calibrated before continuing.
        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.sleep(50);
            op.idle();
        }
        double[] tracker = track();
        xpos = 0;
        ypos = 0;
        op.sleep(500);
        if (tracker[2] > 5 && tracker[2] < -5) {
            globalAngle = 0;
        }
        xpos = 0;
        ypos = 0;
        angle = 0;
    }

    public static float getXpos() {
        return xpos;
    }

    public static float getYpos() {
        return ypos;
    }

    public static void setXpos(float newXpos) {
//        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom[0]=0;
//        odom[1]=0;
//        odom[2]=0;
        xpos = newXpos;
    }

    public static void setYpos(float newYpos) {
        ypos = newYpos;
    }

    public static void setAngle(float newAngle) {
        globalAngle = newAngle + 181;
    }

    public void navigate() {//navigation.navigate(op);
    }

    public void navigateTeleOp() {//navigation.navigateTeleOp(op);
    }

    public void setPosition(float x, float y, float newAngle) {
        xpos = x;
        ypos = y;
        globalAngle = newAngle;
        //navigation.setPosition(x,y,newAngle);
    }

    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public static float getCurrentAngle() {
        return angle;
    }

    public float getAngle() {
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
        globalAngle %= 360;
        if (globalAngle > 270) {
            globalAngle -= 360;
        }
        if (globalAngle < -270) {
            globalAngle += 360;
        }

        lastAngles = angles;

        return -globalAngle % 360;
        //return navigation.getAngle();
    }

    public double[] track() {
        thisTime = runtime.seconds();
        double differtime = thisTime - lastTime;
        double[] data = {0, 0, 0};
        double[] diff = {odomconst[0] * (odom1.getCurrentPosition() - odom[0]), odomconst[1] * (odom2.getCurrentPosition() - odom[1]),
                //motor.getCurrentPosition(), motor.setPower()             motor.setVelocity(), motor.getVelocity
                odomconst[2] * (odom3.getCurrentPosition() - odom[2])};
        lastTime = thisTime;
        velocity[0] = odomconst[0] * diff[0] / differtime;
        velocity[1] = odomconst[1] * diff[1] / differtime;
        velocity[2] = odomconst[2] * diff[2] / differtime;
        odom[0] += odomconst[0] * diff[0];
        odom[1] += odomconst[1] * diff[1];
        odom[2] += odomconst[2] * diff[2];
        double x = cos((getAngle() * Math.PI / 180));
        double y = sin((getAngle() * Math.PI / 180));
        xVelocity = (y * (diff[0] + diff[1]) / (2 * ticks_per_inch) - x * diff[2] / ticks_per_inch) * 1 / differtime;
        yVelocity = (x * (diff[0] + diff[1]) / (2 * ticks_per_inch) + y * diff[2] / ticks_per_inch) * 1 / differtime;
        Velocity=sqrt(xVelocity*xVelocity+yVelocity*yVelocity);
        ypos += yVelocity * differtime;
        xpos += xVelocity * differtime;
        aVelocity=(getAngle()-angle)/differtime;
        angle = getAngle();
        data[0] = xpos;
        data[1] = ypos;
        data[2] = angle;
        op.telemetry.addData("xpos", xpos);
        op.telemetry.addData("ypos", ypos);
        op.telemetry.addData("angle", angle);
        op.telemetry.update();
        return data;
        //return navigation.getPosition();
    }

    public boolean goToPositionTeleop(double y, double x, double a, double power) {

        double f = x;
        x = y;
        y = f;
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        target_position[0] = x;
        target_position[1] = y - 0.15;
        target_position[2] = a;
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        boolean maxspeed = false;
        if (difference > 60 && power > 0.7) {
            maxspeed = true;
        }
        double slowdistance = 22;
        if (maxspeed) {
            slowdistance = 28;
        }
        double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        runtime.reset();
        while (op.opModeIsActive() && (abs(difference) >= 0.5) && !gotoPosition_off) {
            currentPosition = track();
            difftime = op.getRuntime() - time;
            time += difftime;
            diffpos = sqrt((currentPosition[0] - x) * (currentPosition[0] - x) + (currentPosition[1] - y) * (currentPosition[1] - y));
            sped = diffpos / difftime;
            if (sped < 0.5) {
                stoptime += 1;
            } else if (sped > 0.7) {
                stoptime = 0;
            }
            if (stoptime > 300) {
                stopAllMotors();
                return true;
            }
            op.telemetry.addData("time", difftime);
            op.telemetry.addData("sped", sped);
            op.telemetry.addData("distance", difference);
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            if (difference < sped / 2 && difference < 30 && max < 0.3) {
                power = 0.25;
                maxpower = 0.25;
                max = 0.28;
            }
            if (difference > sped / 2) {
                power = startpower;
            }
            if (power > startpower) {
                power = startpower;
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(x, -y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
            anglePower[0] = sin(angleInRadians + PI / 4);
            anglePower[1] = sin(angleInRadians - PI / 4);
            anglecorrection = error * 0.06;
            if (anglecorrection > max) {
                anglecorrection = max;
            }
            if (abs(anglePower[1]) > abs(anglePower[0])) {
                anglePower[1] *= abs(1 / anglePower[1]);
                anglePower[0] *= abs(1 / anglePower[1]);
            } else {
                anglePower[1] *= abs(1 / anglePower[0]);
                anglePower[0] *= abs(1 / anglePower[0]);
            }
            while (power < maxpower) {
                power *= maxpower / abs(power);
            }
            if ((abs(anglePower[0]) + abs(anglePower[1])) < 2 && power < 0.4) {
                double constantinople = 2 / (abs(anglePower[1]) + abs(anglePower[0]));
                power *= constantinople;
            }
            op.telemetry.addData("power", power);
            motorRightBack.setPower((power * anglePower[1] + anglecorrection));
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

            difference = abs(sqrt(x * x + y * y));
            x = currentPosition[0];
            y = currentPosition[1];
        }
        stopAllMotors();
        return true;
    }

    public void goToPosition(double y, double x, double a, double power) {

        double f = x;
        x = y;
        y = f;
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        target_position[0] = x;
        target_position[1] = y - 0.15;
        target_position[2] = a;
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        boolean maxspeed = false;
        if (difference > 60 && power > 0.7) {
            maxspeed = true;
        }
        double slowdistance = 22;
        if (maxspeed) {
            slowdistance = 28;
        }
        double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        runtime.reset();
        while (op.opModeIsActive() && (abs(difference) >= 0.5) && !gotoPosition_off) {
            currentPosition = track();
            difftime = op.getRuntime() - time;
            time += difftime;
            diffpos = sqrt((currentPosition[0] - x) * (currentPosition[0] - x) + (currentPosition[1] - y) * (currentPosition[1] - y));
            sped = diffpos / difftime;
            if (sped < 0.5) {
                stoptime += 1;
            } else if (sped > 0.7) {
                stoptime = 0;
            }
            if (stoptime > 300) {
                stopAllMotors();
                return;
            }
            op.telemetry.addData("time", difftime);
            op.telemetry.addData("sped", sped);
            op.telemetry.addData("distance", difference);
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            if (difference < sped / 2 && difference < 30 && max < 0.3) {
                power = 0.25;
                maxpower = 0.25;
                max = 0.28;
            }
            if (difference > sped / 2) {
                power = startpower;
            }
            if (power > startpower) {
                power = startpower;
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(x, -y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
            anglePower[0] = sin(angleInRadians + PI / 4);
            anglePower[1] = sin(angleInRadians - PI / 4);
            anglecorrection = error * 0.06;
            if (anglecorrection > max) {
                anglecorrection = max;
            }
            if (abs(anglePower[1]) > abs(anglePower[0])) {
                anglePower[1] *= abs(1 / anglePower[1]);
                anglePower[0] *= abs(1 / anglePower[1]);
            } else {
                anglePower[1] *= abs(1 / anglePower[0]);
                anglePower[0] *= abs(1 / anglePower[0]);
            }
            while (power < maxpower) {
                power *= maxpower / abs(power);
            }
            if ((abs(anglePower[0]) + abs(anglePower[1])) < 2 && power < 0.4) {
                double constantinople = 2 / (abs(anglePower[1]) + abs(anglePower[0]));
                power *= constantinople;
            }
            op.telemetry.addData("power", power);
            motorRightBack.setPower((power * anglePower[1] + anglecorrection));
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

            difference = abs(sqrt(x * x + y * y));
            x = currentPosition[0];
            y = currentPosition[1];
        }
        stopAllMotors();
    }

    public void goToPositionWithoutStop(double y, double x, double a, double power) {
        if (!isCorgi) {
            double f = x;
            x = y;
            y = f;
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double[] currentPosition = track();
            double[] target_position = {0, 0, 0};
            double anglecorrection = 0;
            target_position[0] = x;
            target_position[1] = y - 0.15;
            target_position[2] = a;
            double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            double angleInRadians = atan2(x, 1.4 * y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double max = 0.22;
            while (op.opModeIsActive() && (difference >= 1) && op.gamepad2.left_trigger != 1) {
                currentPosition = track();
                if (difference < 10) {
                    power = startpower * difference / 25;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(-x, y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double error = currentPosition[2] - target_position[2];
                error %= 360;
                if (error > 180) {
                    error -= 360;
                }
                if (error < -180) {
                    error += 360;
                }
                anglecorrection = error * -0.05;
                if (anglecorrection > max) {
                    anglecorrection = max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }
                while (abs(power) < 0.42) {
                    power *= 0.42 / abs(power);
                }
                motorRightBack.setPower(1.4 * (power * anglePower[1] + anglecorrection));//1.4 IF YOU ARE USING WALRUS MULTIPLY THIS BY 1.4
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);
                difference = abs(sqrt((x) * (x) + (y) * (y)));
//            op.telemetry.addData("distance", difference);
                op.telemetry.update();
            }
            op.telemetry.addData("done", true);
        } else if (isCorgi) {
            double f = x;
            x = y;
            y = f;
            motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.FORWARD);
            motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightBack.setDirection(DcMotor.Direction.FORWARD);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double[] currentPosition = track();
            double[] target_position = {0, 0, 0};
            double anglecorrection = 0;
            double maxpower = 0.2;
            double time = op.getRuntime();
            double difftime = 0;
            double diffpos = 0;
            double sped = 0;
            double stoptime = 0;
            target_position[0] = x;
            target_position[1] = y - 0.15;
            target_position[2] = a;
            double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            boolean maxspeed = false;
            if (difference > 60 && power > 0.7) {
                maxspeed = true;
            }
            double slowdistance = 22;
            if (maxspeed) {
                slowdistance = 28;
            }
            double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double error = 0;
            double max = 0.15;
            while (op.opModeIsActive() && (abs(difference) >= 3) && !gotoPosition_off) {
                currentPosition = track();
                difftime = op.getRuntime() - time;
                time += difftime;
                diffpos = sqrt((currentPosition[0] - x) * (currentPosition[0] - x) + (currentPosition[1] - y) * (currentPosition[1] - y));
                sped = diffpos / difftime;
                if (sped < 0.5) {
                    stoptime += 1;
                } else if (sped > 0.7) {
                    stoptime = 0;
                }
                if (stoptime > 300) {
                    stopAllMotors();
                    return;
                }
                op.telemetry.addData("time", difftime);
                op.telemetry.addData("sped", sped);
                op.telemetry.addData("distance", difference);
                error = currentPosition[2] - target_position[2];
                error %= 360;
                if (error > 180) {
                    error -= 360;
                }
                if (error < -180) {
                    error += 360;
                }
                if (difference < sped / 3 && difference < 30 && max < 0.3) {
                    power = 0.5;
                    maxpower = 0.25;
                }
                if (difference > sped / 2) {
                    power = startpower;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(x, -y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                anglecorrection = error * 0.06;
                if (anglecorrection > max) {
                    anglecorrection = max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }
                while (power < maxpower) {
                    power *= maxpower / abs(power);
                }
                if ((abs(anglePower[0]) + abs(anglePower[1])) < 2 && power < 0.4) {
                    double constantinople = 2 / (abs(anglePower[1]) + abs(anglePower[0]));
                    power *= constantinople;
                }
                op.telemetry.addData("power", power);
                motorRightBack.setPower((power * anglePower[1] + anglecorrection));
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(x * x + y * y));
                x = currentPosition[0];
                y = currentPosition[1];
            }
        }
    }

    //direction=1 for robot start angle, direction = 0 for backwards
    public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power) {
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double[][] point = {{0,0},{0,0},{0,0},{0,0},{0,0}};
        point[0][0] = x0;
        point[0][1] = y0;
        point[1][0] = x1;
        point[1][1] = y1;
        point[2][0] = x2;
        point[2][1] = y2;
        point[3][0] = x3;
        point[3][1] = y3;
        point[4][0] = x4;
        point[4][1] = y4;
        double[] currentPosition = track();
        double[] startPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        double axisa = atan2(x2 - x1, y2 - y1);
        target_position[0] = x3;
        target_position[1] = y3 - 0.15;
        target_position[2] =0;
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(point[2][0] - point[1][0], point[2][1] - point[1][1]) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        double mpconst = 0;
        double mpyVelocity = 0;
        double[] tarcurpos = {0,0,0};
        runtime.reset();
        double t=0;
        for (int i =-1; i < 0; i++) {
            double timedis = sqrt(pow(point[i + 2][0] - point[i + 1][0], 2) + pow(point[i + 2][1] - point[i + 1][1], 2));
            startPosition = currentPosition;
            axisa=atan2(-point[i+2][1]+point[i+1][1],point[i+2][0]-point[i+1][0]);
//            double looptime=0;
//            double lasteTime=0;
//            double thisetime=0;
            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                //thisetime=runtime.seconds();
                //looptime=thisetime-lasteTime;
                //lasteTime=thisetime;

                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/4)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * ((2 * point[i + 1][1]) + ( + point[i + 2][1]) + 2 * ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                error = currentPosition[2];
                error %= 360;
                double x = target_position[0] - currentPosition[0];
                double y = target_position[1] - currentPosition[1];
                double xError=0;//sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2)))-xVelocity+tarcurpos[0]-currentPosition[0];
                xError*=0.02;
                double yError=0;//(xError+xVelocity*0.02)*mpconst-yVelocity*0.02+(tarcurpos[1]-currentPosition[1])*0.02;
                angleInRadians = atan2(y, -x) - (currentPosition[2] * PI / 180);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-angle/135);
                anglecorrection = -targetaVelocity+aVelocity/135;

                if (anglecorrection > max) {
                    anglecorrection = max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }
                op.telemetry.addData("difference", difference);
                op.telemetry.addData("t", t);
                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
                op.telemetry.addData("angletarget", target_position[2]);
                op.telemetry.addData("angletarget2", angleInRadians);
                op.telemetry.addData("mp", mpconst);
                op.telemetry.addData("error", error);
                op.telemetry.addData("axisa", axisa);
                op.telemetry.addData("xerror", xError);
                op.telemetry.addData("yError", yError);
                op.telemetry.addData("xvelocity", xVelocity);
                op.telemetry.addData("yvelocity", yVelocity);
                op.telemetry.addData("xvelocity",sqrt(pow((power)*30,2)/(1+pow(mpconst,2))));
                op.telemetry.addData("yvelocity", (xError+xVelocity*0.02)*mpconst*50);
                motorRightBack.setPower((power * anglePower[1]-xError+yError + anglecorrection)/3);
                motorRightFront.setPower((power * anglePower[0]+xError+yError + anglecorrection)/3);
                motorLeftBack.setPower((power * anglePower[0]+xError+yError - anglecorrection)/3);
                motorLeftFront.setPower((power * anglePower[1]-xError+yError - anglecorrection)/3);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            }
        }

        for (int i =0; i < 2; i++) {
            double timedis = sqrt(pow(point[i + 2][0] - point[i + 1][0], 2) + pow(point[i + 2][1] - point[i + 1][1], 2));
            startPosition=currentPosition;
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            axisa=atan2(-point[i+2][1]+point[i+1][1],point[i+2][0]-point[i+1][0]);
            t=0;

            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/4)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/10)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i+1][0]) + (-point[i+0][0] + point[i+2][0]) * t + (2 * point[i+0][0] - 5 * point[i+1][0] + 4 * point[i+2][0] - point[i+3][0]) * pow(t, 2) +
                        (-point[i+0][0] + 3 * point[i+1][0] - 3 * point[i+2][0] + point[i+3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i+1][1]) + (-point[i+0][1] + point[i+2][1]) * t + (2 * point[i+0][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * pow(t, 2) +
                        (-point[i+0][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                target_position[2] = (0.5 * ((2 * point[i+1][1]) + (-point[i+0][1] + point[i+2][1]) + 2 * (2 * point[i+0][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * t +
                        3 * (-point[i+0][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                error = currentPosition[2];
                error %= 360;
                double x = target_position[0] - currentPosition[0];
                double y = target_position[1] - currentPosition[1];
                double xError=0;//sqrt(pow((power)*power*30,2)/(1+pow(mpconst,2)))-xVelocity;
                xError*=0.02;
                double yError=0;//(xError+xVelocity*0.02)*mpconst-yVelocity*0.02;
                angleInRadians = atan2(y, -x) - (currentPosition[2] * PI / 180);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-angle/135);
                anglecorrection = -targetaVelocity+aVelocity/135;

                if (anglecorrection > max) {
                    anglecorrection = max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }
                op.telemetry.addData("difference", difference);
                op.telemetry.addData("t", t);
                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
                op.telemetry.addData("angletarget", target_position[2]);
                op.telemetry.addData("angletarget2", angleInRadians);
                op.telemetry.addData("mp", mpconst);
                op.telemetry.addData("error", error);
                op.telemetry.addData("axisa", axisa);
                op.telemetry.addData("xerror", xError);
                op.telemetry.addData("yError", yError);
                op.telemetry.addData("xvelocity", xVelocity);
                op.telemetry.addData("yvelocity", yVelocity);
                op.telemetry.addData("xvelocity",sqrt(pow((power)*30,2)/(1+pow(mpconst,2))));
                op.telemetry.addData("yvelocity", (xError+xVelocity*0.02)*mpconst*50);
                motorRightBack.setPower((power * anglePower[1]-xError+yError + anglecorrection)/3);
                motorRightFront.setPower((power * anglePower[0]+xError+yError + anglecorrection)/3);
                motorLeftBack.setPower((power * anglePower[0]-xError+yError - anglecorrection)/3);
                motorLeftFront.setPower((power * anglePower[1]+xError+yError - anglecorrection)/3);
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            }
        }
        for (int i =2; i < 3; i++) {
            double timedis = sqrt(pow(point[i + 2][0] - point[i + 1][0], 2) + pow(point[i + 2][1] - point[i + 1][1], 2));
            startPosition = currentPosition;
            axisa=atan2(-point[i+2][1]+point[i+1][1],point[i+2][0]-point[i+1][0]);
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            double xError=0;
            double yError=0;
            t=0;
            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/4)/(oneDistance+twoDistance);
                }
                if(t>1){
                    t=1;
                }
                target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] ) * pow(t, 2) +
                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] ) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] ) * pow(t, 2) +
                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] ) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                target_position[2] = (0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) + 2 * (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] ) * t +
                        3 * (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] ) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                error = currentPosition[2];
                error %= 360;
                double x = target_position[0] - currentPosition[0];
                double y = target_position[1] - currentPosition[1];

                    xError = sqrt(pow((power) * power * 30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                    xError *= 0.02;
                    yError = (xError + xVelocity * 0.02) * mpconst - yVelocity * 0.02;
                    xError=0;
                    yError=0;
                double approxDifference=0;

                if(difference<20){
                     t = (oneDistance)/(oneDistance+twoDistance);
                    double tdiff= (1-t)/20;
                    double[] dummyPosition = {0,0};
                    double[] dummyPositionTwo = {currentPosition[0],currentPosition[1]};
                    for(double j=t; j<1+tdiff; j+=tdiff){
                        dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * j + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] ) * pow(j, 2) +
                                (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] ) * pow(j, 3));

                        dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * j + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] ) * pow(j, 2) +
                                (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] ) * pow(j, 3));
                        approxDifference+=abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]),2) + pow(dummyPosition[1] - dummyPositionTwo[1],2)));
                        dummyPositionTwo=dummyPosition;
                    }
                    int xCon=0;
                    if(currentPosition[0]<point[i+2][0]){
                        xCon=1;
                    }
                    else {
                        xCon = -1;
                    }
                    int yCon=0;
                    if(currentPosition[1]<point[i+2][1]){
                        yCon=1;
                    }
                    else{
                        yCon=-1;
                    }
                    //power=startpower;
                    double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                    if(approxDifference<trueVelocity*power/2){
                        //xError*=approxDifference/10;
                        //yError*=approxDifference/10;
                        //power=min((pow(approxDifference,2)/4-trueVelocity)/30,startpower);
                        xError = pow(approxDifference,2)/4 / (1 + pow(mpconst, 2)) - xVelocity;
                        xError *= 0.02;
                        yError = (xError + xVelocity * 0.02) * mpconst - yVelocity * 0.02;
                    }
                    else{
                        power=startpower;
                    }
                }
                angleInRadians = atan2(y, -x) - (currentPosition[2] * PI / 180);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-angle/135);
                anglecorrection = -targetaVelocity+aVelocity/135;

                if (anglecorrection > max) {
                    anglecorrection = max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }

                op.telemetry.addData("difference", difference);
                op.telemetry.addData("t", t);
                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
                op.telemetry.addData("angletarget", target_position[2]);
                op.telemetry.addData("angletarget2", angleInRadians);
                op.telemetry.addData("mp", mpconst);
                op.telemetry.addData("error", error);
                op.telemetry.addData("axisa", axisa);
                op.telemetry.addData("xerror", xError);
                op.telemetry.addData("yError", yError);
                op.telemetry.addData("xvelocity", xVelocity);
                op.telemetry.addData("yvelocity", yVelocity);
                op.telemetry.addData("xvelocity",sqrt(pow((power)*30,2)/(1+pow(mpconst,2))));
                op.telemetry.addData("yvelocity", (xError+xVelocity*0.02)*mpconst*50);
                op.telemetry.addData("yvelocity", approxDifference);

                motorRightBack.setPower((power * anglePower[1]-xError+yError + anglecorrection)/5);
                motorRightFront.setPower((power * anglePower[0]+xError+yError + anglecorrection)/5);
                motorLeftBack.setPower((power * anglePower[0]-xError+yError - anglecorrection)/5);
                motorLeftFront.setPower((power * anglePower[1]+xError+yError - anglecorrection)/5);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            }
        }
        stopAllMotors();


    }

    public void turnInPlace(double target, double power) {
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!isCorgi) {
            float currentAngle = getAngle();
            float newTarget = (float) target;
            float error = (float) target - currentAngle;
            double gain = -0.006;
            int direction = 1;
            if (error < 0) {
                direction = -1;
            } else {
                direction = 1;
            }
            double rightPower = direction * min(abs(power * gain * error), abs(power));
            double leftPower = -rightPower;

            error = newTarget - currentAngle;
            if (newTarget > 270) {
                newTarget -= 360;
            }
            if (newTarget < -270) {
                newTarget += 360;
            }
            error = newTarget - currentAngle;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }

            if (abs(error) < 20) {
                while (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                    currentAngle = (float) track()[2];
                    if (newTarget > 270) {
                        newTarget -= 360;
                    }
                    if (newTarget < -270) {
                        newTarget += 360;
                    }
                    error = newTarget - currentAngle;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    if (error < 0) {
                        direction = -1;
                    } else {
                        direction = 1;
                    }
                    rightPower = direction * min(abs(power * gain * error), abs(power));
                    leftPower = -rightPower;

                    if (abs(leftPower) < 0.19) {
                        leftPower *= 0.18 / abs(leftPower);
                    }
                    if (abs(rightPower) < 0.18) {
                        rightPower *= 0.18 / abs(rightPower);
                    }
                    motorLeftBack.setPower(leftPower);
                    motorLeftFront.setPower(leftPower);
                    motorRightBack.setPower(rightPower);
                    motorRightFront.setPower(rightPower);
                }

                motorLeftBack.setPower(0);
                motorRightFront.setPower(0);
                motorLeftFront.setPower(0);
                motorRightBack.setPower(0);
            }
        } else {
            float currentAngle = getAngle();
            float newTarget = (float) target;
            float error = (float) target - currentAngle;
            double gain = -0.03;
            int direction = 1;
            if (error < 0) {
                direction = -1;
            } else {
                direction = 1;
            }
            double rightPower = direction * min(abs(power * gain * error), abs(power));
            double leftPower = -rightPower;


            if (newTarget > 180) {
                newTarget = newTarget - 360;
            }
            if (newTarget <= -180) {
                newTarget = newTarget + 360;
            }
            while (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                currentAngle = (float) track()[2];
                error = newTarget - currentAngle;
                if (error < 0) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                rightPower = -direction * min(abs(power * gain * error), abs(power));
                if (abs(rightPower) < 0.16) {
                    rightPower *= 0.16 / abs(rightPower);
                }
                leftPower = -rightPower;
                motorLeftBack.setPower(leftPower);
                motorLeftFront.setPower(leftPower);
                motorRightBack.setPower(rightPower);
                motorRightFront.setPower(rightPower);
            }

            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
            motorLeftFront.setPower(0);
        }
    }


    public void moveForward(double distance, double power) {
        double x = sin(getAngle() * PI / 180) * distance, y = cos(getAngle() * PI / 180) * distance;
        moveAngle(x, y, power);
    }

    public void moveBackward(double distance, double power) {
        double x = -sin(getAngle() * PI / 180) * distance, y = -cos(getAngle() * PI / 180) * distance;
        moveAngle(x, y, power);
    }

    public void moveRight(double distance, double power) {//right is positive use distance to change direction
        double x = cos(getAngle() * PI / 180) * distance, y = sin(getAngle() * PI / 180) * distance;
        moveAngle(x, y, power);
    }

    public void moveLeft(double distance, double power) {

        double x = -cos(getAngle() * PI / 180) * distance, y = -sin(getAngle() * PI / 180) * distance;
        moveAngle(x, y, power);
    }

    public void moveAngle(double x, double y, double power) {//anglecorrection
        /*motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        try {
            //Create File
            File myFTCfile = new File("/storage/emulated/0/tmp/OdometryTest.csv");
            if (myFTCfile.createNewFile()) {
                op.telemetry.addData("moveAngleOdometry:", "File created:%S\n", "Odomeytry");
                op.telemetry.update();
            } else {
                op.telemetry.addData("moveAngleOdometry:", "File already exists:%S\n", "Odometry");
                op.telemetry.update();
            }
            FileWriter wFTCfile = new FileWriter(myFTCfile);

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
                /*if (difference < 5) {
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
                anglecorrection = (currentPosition[2] - target_position[2]) * 0.005;
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
                //op.sleep(3000);
                //FileWriteHandle;
                wFTCfile.write(System.currentTimeMillis() + "," + String.format("%.2f", currentPosition[0]) + "," + String.format("%.2f", currentPosition[1]) + "," +
                        String.format("%.2f", currentPosition[2]) + "," +
                        String.format("%.2f", power) + "," +
                        String.format("%.2f", anglePower[0]) + "," +
                        String.format("%.2f", anglePower[1]) + "," +
                        String.format("%.2f", anglecorrection) + "," +
                        String.format("%.2f", difference) + "," + "\n" +
                        String.format("%.2f", encoder[0]) + "," + String.format("%.2f", encoder[1]) + "," + String.format("%.2f", encoder[2]) + "," + String.format("%.2f", encoder[3]) + "," + "\n");
            }
            wFTCfile.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        turnInPlace(startAngle, 0.5);*/
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startAngle = getAngle();
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0, startx = x, starty = y, calculations = 0;
        target_position[0] = currentPosition[0] + y;
        target_position[1] = currentPosition[1] + x - 0.15;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        int gamernum = 0;
        while (op.opModeIsActive() && (difference > 0.75)) {
            currentPosition = track();
            power = difference / 15;
            if (power > startpower) {
                power = startpower;
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(x, y) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
            anglePower[0] = sin(angleInRadians + PI / 4);
            anglePower[1] = sin(angleInRadians - PI / 4);
            anglecorrection = (currentPosition[2] - target_position[2]) * 0.05;
            if (abs(anglePower[1]) > abs(anglePower[0])) {
                anglePower[1] *= abs(1 / anglePower[1]);
                anglePower[0] *= abs(1 / anglePower[1]);
            } else {
                anglePower[1] *= abs(1 / anglePower[0]);
                anglePower[0] *= abs(1 / anglePower[0]);
            }
            if ((abs(power * anglePower[1] + anglecorrection) <= 0.2 && abs(power * anglePower[0] - anglecorrection) <= 0.2) || (abs(power * anglePower[0] + anglecorrection) <= 0.2 && abs(power * anglePower[1] - anglecorrection) <= 0.2)) {
                anglePower[1] *= 1.5;
                anglePower[0] *= 1.5;
            }
            while (abs(power) < 0.4) {
                power *= 0.4 / abs(power);
            }
            motorRightBack.setPower(power * anglePower[1] + anglecorrection);
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
            difference = abs(sqrt((x) * (x) + (y) * (y)));
            op.telemetry.addData("distance", difference);
            op.telemetry.update();
        }
        currentPosition = track();
        turnInPlace(startAngle, 1.0);
        stopAllMotors();
    }

    public int getMultiplier(DcMotorEx motor) {
        if (motor == odom1) {
            return odomconst[0];
        } else if (motor == odom2) {
            return odomconst[1];
        } else if (motor == odom3) {
            return odomconst[2];
        } else {
            return 1;
        }
    }

    public double getRawVelocity(DcMotorEx motor) {
        int multiplier = getMultiplier(motor);
        return motor.getVelocity() * multiplier;
    }

    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    public double getCorrectedVelocity(DcMotorEx motor) {
        if (motor == odom1) {
            return inverseOverflow(getRawVelocity(motor), velocity[0]);
        } else if (motor == odom2) {
            return inverseOverflow(getRawVelocity(motor), velocity[1]);
        } else if (motor == odom3) {
            return inverseOverflow(getRawVelocity(motor), velocity[2]);
        } else {
            return getRawVelocity(motor);
        }
    }
}
