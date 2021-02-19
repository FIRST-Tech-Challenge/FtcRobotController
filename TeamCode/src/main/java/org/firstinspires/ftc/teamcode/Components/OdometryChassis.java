package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.Navigations.Navigation;
import org.firstinspires.ftc.teamcode.Components.Navigations.Odometry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
//2.0,1.7,1.1
public class OdometryChassis extends BasicChassis {
    private Navigation navigation= null;
    DcMotorEx odom1;
    DcMotorEx odom2;
    DcMotorEx odom3;
    int[] odomconst = {-1,1,-1};
    float ticks_per_inch = (float)(8640*2.54/38*Math.PI)*72/76;
    float robot_diameter = (float)sqrt(619.84);
    float[] odom = new float[3];
    private LinearOpMode op = null;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private float globalAngle;
    double power = .30, correction;
    float xpos,ypos,angle;



    //set true to enable imu vice versa
    final boolean enableIMU = true;

    public OdometryChassis(LinearOpMode opMode,boolean navigator) {
        super(opMode);
        op = opMode;

        // Chassis encoders
        odom1 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        odom3 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        odom2 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
        // reset encoder count.
        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        //navigation = new Navigation(op);
    }
    public void navigate(){//navigation.navigate(op);
        }
    public void navigateTeleOp(){//navigation.navigateTeleOp(op)
        }
    public void setPosition(double x, double y, double angle){
        //navigation.setPosition(x,y,angle);
    }
    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
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

        lastAngles = angles;

        return -globalAngle;
    }
    public double[] track() {
        double data[]={0,0,0};
double diff[]={odomconst[0]*(odom1.getCurrentPosition() - odom[0]),odomconst[1]*(odom2.getCurrentPosition() - odom[1]),odomconst[2]*(odom3.getCurrentPosition() - odom[2])};
odom[0] += odomconst[0]*diff[0];
odom[1] += odomconst[1]*diff[1];
odom[2] += odomconst[2]*diff[2];
double x =  cos((getAngle() * Math.PI / 180));
double y = sin((getAngle() * Math.PI / 180));
ypos += (y * (diff[0]+diff[1])/(2*ticks_per_inch) - x * diff[2]/ticks_per_inch)*1;
xpos += (x * (diff[0]+diff[1])/(2*ticks_per_inch) + y * diff[2]/ticks_per_inch)*1;
angle=getAngle();
op.telemetry.addData("x",xpos);
op.telemetry.addData("y",ypos);
//op.telemetry.addData("odom1",odomconst[0]*odom1.getCurrentPosition());
//op.telemetry.addData("odom2",odomconst[1]*odom2.getCurrentPosition());
//op.telemetry.addData("odom3",odomconst[2]*odom3.getCurrentPosition());
op.telemetry.addData("angle",angle);
op.telemetry.update();
data[0]=xpos;
data[1]=ypos;
data[2]=angle;
        op.telemetry.addData("x", xpos);
        op.telemetry.addData("y", ypos);
        op.telemetry.addData("angle", angle);
        op.telemetry.update();
return data;
    }
    public void goToPosition(double x, double y, double a, double power){
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
        double anglecorrection=0;
        target_position[0] = x;
        target_position[1] = y-0.15;
        target_position[2] = a;
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower=power;
        while (op.opModeIsActive() && (difference >= 1)) {
            currentPosition = track();
            power=difference/15;
            if(power>startpower){
                power=startpower;
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(x, y) - (target_position[2]+((currentPosition[2] * PI / 180)-target_position[2])/1);
            anglePower[0] = sin(angleInRadians + PI / 4);
            anglePower[1] = sin(angleInRadians - PI / 4);
            anglecorrection = (currentPosition[2] - target_position[2])%360 * 0.05;
            if (abs(anglePower[1]) > abs(anglePower[0])) {
                anglePower[1] *= abs(1 / anglePower[1]);
                anglePower[0] *= abs(1 / anglePower[1]);
            } else {
                anglePower[1] *= abs(1 / anglePower[0]);
                anglePower[0] *= abs(1 / anglePower[0]);
            }
//            if((abs(power * anglePower[1] + anglecorrection)<=0.2&&abs(power * anglePower[0] - anglecorrection)<=0.2)||(abs(power * anglePower[0] + anglecorrection)<=0.2&&abs(power * anglePower[1] - anglecorrection)<=0.2)){
//                anglePower[1]*=1.5;
//                anglePower[0]*=1.5;
//            }
            while(abs(power)<0.35){
                power*=0.35/abs(power);
            }
            motorRightBack.setPower(power * anglePower[1] + anglecorrection);
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
            difference = abs(sqrt((x) * (x) + (y) * (y)));
            op.telemetry.addData("distance", difference);
        }
        stopAllMotors();
        turnInPlace(a,1.0);
        stopAllMotors();
        op.telemetry.addData("done", true);
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

        float currentAngle = getAngle();
        float newTarget = (float)target;
        float error = (float)target-currentAngle;
        double gain = 0.05;
        int direction=1;
        if(error<0){
            direction = -1;
        }else{
            direction = 1;
        }
        double rightPower = direction*min(abs(power*gain*error),abs(power));
        double leftPower = -rightPower;


        if (newTarget>180){newTarget=newTarget-360;}
        if (newTarget<=-180){newTarget=newTarget+360;}

        if(abs(error)<10){
            while (op.opModeIsActive() && (error > 0.25  || error < -0.25))
            {
                currentAngle = (float)track()[2];
                error = newTarget - currentAngle;
                if(error<0){
                    direction = -1;
                }else{
                    direction = 1;
                }
                rightPower = -direction*min(abs(power*gain*error),abs(power));
                leftPower = -rightPower;
                if(abs(leftPower)<0.315){
                    leftPower*=0.3/abs(leftPower);
                }
                if(abs(rightPower)<0.315){
                    rightPower*=0.3/abs(rightPower);
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
        else {
            while (op.opModeIsActive() && (error > 0.5 || error < -0.5)) {
                currentAngle = getAngle();
                error = newTarget - currentAngle % 360;
                if (error < 0) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                rightPower = -direction * min(abs(power * gain * error), abs(power));
                leftPower = -rightPower;
                while (abs(leftPower) < 0.4) {
                    leftPower *= 0.4 / abs(leftPower);
                }
                while (abs(rightPower) < 0.4) {
                    rightPower *= 0.4 / abs(rightPower);
                }
                op.telemetry.addData("leftPower", leftPower);
                op.telemetry.addData("rightPower", rightPower);
                op.telemetry.addData("error", error);
                motorLeftBack.setPower(leftPower);
                motorLeftFront.setPower(leftPower);
                motorRightBack.setPower(rightPower);
                motorRightFront.setPower(rightPower);
                track();
            }

            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorLeftFront.setPower(0);
            motorRightBack.setPower(0);
        }
    }

    public void moveForward(double distance, double power) {
        double x=sin(getAngle()*PI/180)*distance,y=cos(getAngle()*PI/180)*distance;
        moveAngle(x,y,power);
    }

    public void moveBackward(double distance, double power) {
        double x=-sin(getAngle()*PI/180)*distance,y=-cos(getAngle()*PI/180)*distance;
        moveAngle(x,y,power);
    }

    public void moveRight(double distance, double power) {//right is positive use distance to change direction
        double x=cos(getAngle()*PI/180)*distance,y=sin(getAngle()*PI/180)*distance;
        moveAngle(x,y,power);
    }

    public void moveLeft(double distance, double power) {

        double x=-cos(getAngle()*PI/180)*distance,y=-sin(getAngle()*PI/180)*distance;
        moveAngle(x,y,power);
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
        double anglecorrection=0,startx=x,starty=y,calculations=0;
        target_position[0] = currentPosition[0] + y;
        target_position[1] = currentPosition[1] + x-0.15;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower=power;
        int gamernum = 0;
        while (op.opModeIsActive() && (difference>0.75)) {
            currentPosition = track();
            power=difference/15;
            if(power>startpower){
                power=startpower;
            }
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            angleInRadians = atan2(x, y) - (target_position[2]+((currentPosition[2] * PI / 180)-target_position[2])/1);
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
                if((abs(power * anglePower[1] + anglecorrection)<=0.2&&abs(power * anglePower[0] - anglecorrection)<=0.2)||(abs(power * anglePower[0] + anglecorrection)<=0.2&&abs(power * anglePower[1] - anglecorrection)<=0.2)){
                    anglePower[1]*=1.5;
                    anglePower[0]*=1.5;
                }
            while(abs(power)<0.4){
                power*=0.4/abs(power);
            }
            motorRightBack.setPower(power * anglePower[1] + anglecorrection);
            motorRightFront.setPower(power * anglePower[0] + anglecorrection);
            motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
            motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
            difference = abs(sqrt((x) * (x) + (y) * (y)));
            op.telemetry.addData("distance", difference);
        }
        currentPosition = track();
        turnInPlace(startAngle,1.0);
        stopAllMotors();
    }
}
