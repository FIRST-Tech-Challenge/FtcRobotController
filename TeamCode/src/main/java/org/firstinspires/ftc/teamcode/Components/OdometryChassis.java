package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
//2.0,1.7,1.1
public class OdometryChassis extends BasicChassis {
   VuforiaWebcam vuforia = null;
    static DcMotorEx odom1;
    static DcMotorEx odom2;
    static DcMotorEx odom3;
    public static final boolean gotoPosition_off=false;
    public static boolean vuforia_on=false;
    final int[] odomconst = {-1,1,-1};
    final float ticks_per_inch = (float)(8640*2.54/38*Math.PI)*72/76;
    float robot_diameter = (float)sqrt(619.84);
    static final float[] odom = new float[3];
    private LinearOpMode op = null;
    private final BNO055IMU imu;
    private Orientation lastAngles=null;
    public static float globalAngle=0;
    public static float xpos=0;
    public static float ypos=0;
    public static float angle;
    double power = .30, correction;


    //set true to enable imu vice versa
    final boolean isCorgi;

    public OdometryChassis(LinearOpMode opMode,boolean navigator, boolean tobeCorgiornottobeCorgi) {
        super(opMode);
        op = opMode;
        xpos=0;
        ypos=0;
        angle=0;
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
        }         if (navigator) {
                vuforia = new VuforiaWebcam(op);
                vuforia.start();
            }
        track();
        xpos=0;
        ypos=0;
        isCorgi=tobeCorgiornottobeCorgi;
        op.sleep(500);
        if(track()[2]>5&&track()[2]<-5){
            globalAngle=0;
        }
        xpos=0;
        ypos=0;
        angle=0;
    }
    public static float getXpos(){
        return xpos;
    }
    public  static float getYpos(){
        return ypos;
    }
    public static void setXpos(float newXpos){
//        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odom1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odom[0]=0;
//        odom[1]=0;
//        odom[2]=0;
        xpos=newXpos;
    }
    public  static void setYpos(float newYpos){
        ypos=newYpos;
    }
    public static void setAngle(float newAngle){
        globalAngle=newAngle+181;
    }
    public void navigate(){//navigation.navigate(op);
        }
    public void navigateTeleOp(){//navigation.navigateTeleOp(op);
        }
    public void setPosition(float x, float y, float newAngle){
        xpos=x;
        ypos=y;
        globalAngle=newAngle;
        //navigation.setPosition(x,y,newAngle);
    }
    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public static float getCurrentAngle(){
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
        globalAngle%=360;
        if(globalAngle>270){
            globalAngle-=360;
        }
        if(globalAngle<-270){
            globalAngle+=360;
        }

        lastAngles = angles;

        return -globalAngle%360;
        //return navigation.getAngle();
    }
    public double[] track() {
        double[] data ={0,0,0};
        double[] diff ={odomconst[0]*(odom1.getCurrentPosition() - odom[0]),odomconst[1]*(odom2.getCurrentPosition() - odom[1]),
                odomconst[2]*(odom3.getCurrentPosition() - odom[2])};
        odom[0] += odomconst[0]*diff[0];
        odom[1] += odomconst[1]*diff[1];
        odom[2] += odomconst[2]*diff[2];
        double x =  cos((getAngle() * Math.PI / 180));
        double y = sin((getAngle() * Math.PI / 180));
        ypos += (y * (diff[0]+diff[1])/(2*ticks_per_inch) - x * diff[2]/ticks_per_inch)*1;
        xpos += (x * (diff[0]+diff[1])/(2*ticks_per_inch) + y * diff[2]/ticks_per_inch)*1;
        angle=getAngle();
        data[0]=xpos;
        data[1]=ypos;
        data[2]=angle;
        op.telemetry.addData("xpos",xpos);
        op.telemetry.addData("ypos",ypos);
        op.telemetry.addData("angle",angle);
        op.telemetry.update();
        return data;
        //return navigation.getPosition();
    }
    public boolean goToPositionTeleop(double y, double x, double a, double power) {

        if(!isCorgi) {
            double f = x;
            x=y;
            y=f;
            double[] currentPosition = track();
            double[] target_position = {x, y-0.15, a};
            double anglecorrection = 0;
            double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            double angleInRadians = atan2(x, 1.4*y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double max = 0.22;
            double error = currentPosition[2]- target_position[2];
            if(op.opModeIsActive() && (difference >= 1)&&!gotoPosition_off) {
                if(difference<15){
                    power = startpower*difference / 25;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(-x, y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                error%=360;
                if(error>180){
                    error-=360;
                }
                if(error<-180){
                    error+=360;
                }
                anglecorrection = error * -0.01;
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
            }
            else if(difference<=1){
                if(error<5){
                    return true;
                }
                else{
                    turnInPlace(a,0.5);
                }
            }
        }
        if(isCorgi) {
            double f = x;
            x=y;
            y=f;
            double[] currentPosition = track();
            double[] target_position = {x, y-0.15, a};
            double anglecorrection = 0;
            double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            double angleInRadians = atan2(x, 1.4*y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double max = 0.22;
            double error = currentPosition[2]- target_position[2];
            error%=360;
            if(error>180){
                error-=360;
            }
            if(error<-180){
                error+=360;
            }
            if(op.opModeIsActive() && (difference >= 1)&&!gotoPosition_off) {
                power = difference*startpower / 30;
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(x, -y*2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
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
                while (abs(power) < 0.25) {
                    power *= 0.25 / abs(power);
                }
                motorRightBack.setPower((power * anglePower[1] + anglecorrection));
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
            }
            else if(difference<=1){
                if(error<5){
                    return true;
                }
                else{
                    turnInPlace(a,0.5);
                }
            }
        }
        return false;
    }
    public void goToPosition(double y, double x, double a, double power){
        if(!isCorgi) {
            double f = x;
            x=y;
            y=f;
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
            double angleInRadians = atan2(x, 1.4*y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double max = 0.22;
            while (op.opModeIsActive() && (difference >= 1)&&op.gamepad2.left_trigger!=1) {
                currentPosition = track();
                if(difference<10){
                    power = startpower*difference / 25;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(-x, y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double error = currentPosition[2]- target_position[2];
                error%=360;
                if(error>180){
                    error-=360;
                }
                if(error<-180){
                    error+=360;
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
            stopAllMotors();
            op.telemetry.addData("done", true);
        }
        else if(isCorgi) {
            double f = x;
            x=y;
            y=f;
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
            double maxpower=0.2;
            double time=op.getRuntime();
            double difftime=0;
            double diffpos=0;
            double sped=0;
            double stoptime=0;
            target_position[0] = x;
            target_position[1] = y - 0.15;
            target_position[2] = a;
            double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            boolean maxspeed=false;
            if(difference>60&&power>0.7){
                maxspeed=true;
            }
            double slowdistance=22;
            if(maxspeed){
                slowdistance=28;
            }
            double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double error=0;
            double max = 0.15;
            while (op.opModeIsActive() && (abs(difference) >= 0.5)&&!gotoPosition_off) {
                currentPosition = track();
                difftime=op.getRuntime()-time;
                time+=difftime;
                diffpos=sqrt((currentPosition[0]-x)*(currentPosition[0]-x)+(currentPosition[1]-y)*(currentPosition[1]-y));
                sped=diffpos/difftime;
                if(sped<0.05){
                    stoptime+=1;
                }
                else if(sped>0.2){
                    stoptime=0;
                }
                if(stoptime>500){
                    stopAllMotors();
                    return;
                }
                op.telemetry.addData("time",difftime);
                op.telemetry.addData("sped", sped);
                op.telemetry.addData("distance", difference);
                error = currentPosition[2]- target_position[2];
                error%=360;
                if(error>180){
                    error-=360;
                }
                if(error<-180){
                    error+=360;
                }
                if(difference<sped/2&&difference<30&&max<0.3){
                    power=0.25;
                    maxpower=0.25;
                    max=0.28;
                }
                if(difference>sped/2){
                    power=startpower;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(x, -y*2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                anglecorrection = error*0.06;
                if(anglecorrection>max){
                    anglecorrection=max;
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
                if((abs(anglePower[0])+abs(anglePower[1]))<2&&power<0.4){
                    double constantinople=2/(abs(anglePower[1])+abs(anglePower[0]));
                    power*=constantinople;
                }
                op.telemetry.addData("power",power);
                motorRightBack.setPower((power * anglePower[1] + anglecorrection));
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                    difference=abs(sqrt(x*x+y*y));
                    x=currentPosition[0];
                    y=currentPosition[1];
            }
            stopAllMotors();
        }
    }public void goToPositionWithoutStop(double y, double x, double a, double power){
        if(!isCorgi) {
            double f = x;
            x=y;
            y=f;
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
            double angleInRadians = atan2(x, 1.4*y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double max = 0.22;
            while (op.opModeIsActive() && (difference >= 1)&&op.gamepad2.left_trigger!=1) {
                currentPosition = track();
                if(difference<10){
                    power = startpower*difference / 25;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(-x, y * 2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double error = currentPosition[2]- target_position[2];
                error%=360;
                if(error>180){
                    error-=360;
                }
                if(error<-180){
                    error+=360;
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
        }
        else if(isCorgi) {
            double f = x;
            x=y;
            y=f;
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
            double angleInRadians = atan2(x, y) - getAngle() * PI / 180;
            double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
            double startpower = power;
            double error=0;
            double max = 0.25;
            while (op.opModeIsActive() && (difference >= 1)&&!gotoPosition_off) {
                currentPosition = track();
                error = currentPosition[2]- target_position[2];
                error%=360;
                if(error>180){
                    error-=360;
                }
                if(error<-180){
                    error+=360;
                }
                if(difference<5){
                    power=0.5;
                }
                if (power > startpower) {
                    power = startpower;
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(x, -y*2) - (target_position[2] + ((currentPosition[2] * PI / 180) - target_position[2]) / 1);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                anglecorrection = error*0.06;
                if(anglecorrection>max){
                    anglecorrection=max;
                }
                if (abs(anglePower[1]) > abs(anglePower[0])) {
                    anglePower[1] *= abs(1 / anglePower[1]);
                    anglePower[0] *= abs(1 / anglePower[1]);
                } else {
                    anglePower[1] *= abs(1 / anglePower[0]);
                    anglePower[0] *= abs(1 / anglePower[0]);
                }
                while (abs(power) < 0.2) {
                    power *= 0.2 / abs(power);
                }
                motorRightBack.setPower((power * anglePower[1] + anglecorrection));
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
            //turnInPlace(a, 0.5);
            op.telemetry.addData("done", true);
        }
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
        if(!isCorgi) {
            float currentAngle = getAngle();
            float newTarget = (float) target;
            float error = (float) target - currentAngle;
            double gain = -0.009;
            int direction = 1;
            if (error < 0) {
                direction = -1;
            } else {
                direction = 1;
            }
            double rightPower = direction * min(abs(power * gain * error), abs(power));
            double leftPower = -rightPower;

            error=newTarget-currentAngle;
            if(newTarget>270){
                newTarget-=360;
            }
            if(newTarget<-270){
                newTarget+=360;
            }
            error = newTarget - currentAngle;
            if(error>180){
                error-=360;
            }
            if(error<-180){
                error+=360;
            }

            if (abs(error) < 20) {
                while (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                    currentAngle = (float) track()[2];
                    if(newTarget>270){
                        newTarget-=360;
                    }
                    if(newTarget<-270){
                        newTarget+=360;
                    }
                    error = newTarget - currentAngle;
                    if(error>180){
                        error-=360;
                    }
                    if(error<-180){
                        error+=360;
                    }
                    if (error < 0) {
                        direction = -1;
                    } else {
                        direction = 1;
                    }
                    rightPower = direction * min(abs(power * gain * error), abs(power));
                    leftPower = -rightPower;

                        if (abs(leftPower) < 0.18) {
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
        }
        else{
        float currentAngle = getAngle();
        float newTarget = (float)target;
        float error = (float)target-currentAngle;
        double gain = -0.06;
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
            while (op.opModeIsActive() && (error > 0.2  || error < -0.2))
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
                    if (abs(leftPower) < 0.14) {
                        leftPower *= 0.14 / abs(leftPower);
                    }
                    if (abs(rightPower) < 0.14) {
                        rightPower *= 0.14 / abs(rightPower);
                    }
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
    public boolean turnInPlaceTeleop(double target, double power) {
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(!isCorgi) {
            float currentAngle = getAngle();
            float newTarget = (float) target;
            float error = (float) target - currentAngle;
            double gain = -0.005;
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
            if(error>180){
                error-=360;
            }
            if(error<-180){
                error+=360;
            }

            if (abs(error) < 20) {
                gain /= 2;
                if (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                    currentAngle = (float) track()[2];
                    error = newTarget - currentAngle;
                    if (error < 0) {
                        direction = -1;
                    } else {
                        direction = 1;
                    }
                    rightPower = direction * min(abs(power * gain * error), abs(power));
                    leftPower = -rightPower;

                    if (abs(leftPower) < 0.18) {
                        leftPower *= 0.18 / abs(leftPower);
                    }
                    if (abs(rightPower) < 0.18) {
                        rightPower *= 0.18 / abs(rightPower);
                    }
                    motorLeftBack.setPower(leftPower);
                    motorLeftFront.setPower(leftPower);
                    motorRightBack.setPower(rightPower);
                    motorRightFront.setPower(rightPower);
                } else {
                    stopAllMotors();
                    return true;
                }
            }
                else{
                    if (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                        currentAngle = (float) track()[2];
                        error = newTarget - currentAngle;
                        if (error < 0) {
                            direction = -1;
                        } else {
                            direction = 1;
                        }
                        rightPower = direction * min(abs(power * gain * error), abs(power));
                        leftPower = -rightPower;

                        if (abs(leftPower) < 0.18) {
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
                    else{
                        stopAllMotors();
                        return true;
                    }
            }
        }
        else{
            float currentAngle = getAngle();
            float newTarget = (float)target;
            float error = (float)target-currentAngle;
            double gain = -0.005;
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

            if(abs(error)<20) {
                gain/=2;
                if (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                    currentAngle = (float) track()[2];
                    error = newTarget - currentAngle;
                    if (error < 0) {
                        direction = -1;
                    } else {
                        direction = 1;
                    }
                    rightPower = -direction * min(abs(power * gain * error), abs(power));
                    leftPower = -rightPower;
                    if (abs(leftPower) < 0.1) {
                        leftPower *= 0.1 / abs(leftPower);
                    }
                    if (abs(rightPower) < 0.1) {
                        rightPower *= 0.1 / abs(rightPower);
                    }
                    motorLeftBack.setPower(leftPower);
                    motorLeftFront.setPower(leftPower);
                    motorRightBack.setPower(rightPower);
                    motorRightFront.setPower(rightPower);
                } else {
                    stopAllMotors();
                    return true;
                }
            }
            else{
                if (op.opModeIsActive() && (error > 0.2 || error < -0.2)) {
                    currentAngle = (float) track()[2];
                    error = newTarget - currentAngle;
                    if (error < 0) {
                        direction = -1;
                    } else {
                        direction = 1;
                    }
                    rightPower = -direction * min(abs(power * gain * error), abs(power));
                    leftPower = -rightPower;
                    if (abs(leftPower) < 0.1) {
                        leftPower *= 0.1 / abs(leftPower);
                    }
                    if (abs(rightPower) < 0.1) {
                        rightPower *= 0.1 / abs(rightPower);
                    }
                    motorLeftBack.setPower(leftPower);
                    motorLeftFront.setPower(leftPower);
                    motorRightBack.setPower(rightPower);
                    motorRightFront.setPower(rightPower);
                } else {
                    stopAllMotors();
                    return true;
                }
            }
        }
        return false;
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
            op.telemetry.update();
        }
        currentPosition = track();
        turnInPlace(startAngle,1.0);
        stopAllMotors();
    }
}
