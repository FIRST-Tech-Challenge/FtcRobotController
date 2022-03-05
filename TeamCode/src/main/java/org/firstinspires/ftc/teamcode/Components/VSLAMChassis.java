package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

//2.0,1.7,1.1
public class VSLAMChassis extends BasicChassis {
    VuforiaThread vuforia = null;
    public static T265Camera slamra;
    double deltaAngle=0;
    double xVelocity = 0;
    double yVelocity = 0;
    double aVelocity = 0;
    double Velocity=0;
    double maxVelocity=0;
    boolean bad = false;
    public static final boolean vuforia_on = false;
    float robot_diameter = (float) sqrt(619.84);
    static final float[] odom = {0f,0f,0f};
    private LinearOpMode op = null;
    Lock location = new ReentrantLock();
    private Orientation lastAngles = null;
    public static float globalAngle = 0;
    public static float xpos = 0;
    public static float ypos = 0;
    public static float angle=0;
    double differtime=0.002;
    final double[] velocity = {0,0,0};
    double power = .30, correction;


    //set true to enable imu vice versa
    final boolean isCorgi=true;
    final ElapsedTime runtime = new ElapsedTime();
    double lastTime = 0;
    double thisTime = 0;

    public VSLAMChassis(LinearOpMode opMode, boolean navigator, boolean tobeCorgiornottobeCorgi) {
        super(opMode);
        op = opMode;
        xpos = 0;
        ypos = 0;
        angle = 0;
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 1, op.hardwareMap.appContext);
        }
        if(navigator) {
            vuforia = new VuforiaThread(op, location);
        }
        op.sleep(1000);
        if(!slamra.isStarted()) {
            slamra.start();
        }

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
        Translation2d startPose = new Translation2d(xpos,ypos);
        Rotation2d startAng = new Rotation2d(newAngle);
    }

    public void navigate() {//navigation.navigate(op);
    }

    public void navigateTeleOp() {//navigation.navigateTeleOp(op);
    }

    public void setPosition(float x, float y, float newAngle) {
        track();
        Translation2d startPose = new Translation2d(x,y);
        Rotation2d startAng = new Rotation2d(newAngle);
        slamra.setPose(new Pose2d(startPose,startAng));
        xpos = x;
        ypos = y;
        angle = newAngle;
        track();
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
        return angle;
    }

    public double[] track() {
        thisTime = runtime.seconds();
        differtime = thisTime - lastTime;
        lastTime = thisTime;
        double newAngle =0;
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up != null) {
            xpos = -(float)(up.pose.getX()/ 0.0254);
            ypos = -(float)(up.pose.getY()/ 0.0254);
            angle = -(float)(up.pose.getHeading()*180/PI);
            xVelocity = -(float)(up.velocity.vxMetersPerSecond/ 0.0254);
            yVelocity = -(float)(up.velocity.vyMetersPerSecond/ 0.0254);
            Velocity = sqrt(xVelocity*xVelocity+yVelocity*yVelocity);
            aVelocity = -(float)up.velocity.omegaRadiansPerSecond*180/PI;
            if(Velocity>maxVelocity){
                maxVelocity=Velocity;
            }
        }
        op.telemetry.addData("xpos", xpos);
        op.telemetry.addData("ypos", ypos);
        op.telemetry.addData("angle", angle);
        op.telemetry.addData("aVelocity", aVelocity);
        op.telemetry.addData("differtime", differtime);
        op.telemetry.addData("maxVel", maxVelocity);
        if(bad){
            op.telemetry.addData("BAD", true);
        }
        op.telemetry.update();
        double[] post = {xpos,ypos,angle};
        return post;
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
        target_position[1] = y;
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
        double startTime = op.getRuntime();
        while (op.opModeIsActive() && (abs(difference) >= 0.5) ) {
            currentPosition = track();
            time = op.getRuntime()-startTime;
            x=target_position[0]-x;
            difference = abs(x);
            if(difference>10) {
                motorRightBack.setPower(power);
                motorRightFront.setPower(power);
                motorLeftBack.setPower(power);
                motorLeftFront.setPower(power);
            }
            else{
                double controlboy = pow((10-difference)/10,2);
                motorRightBack.setPower(power*(1-controlboy));
                motorRightFront.setPower(power*(1-controlboy));
                motorLeftBack.setPower(power*(1-controlboy));
                motorLeftFront.setPower(power*(1-controlboy));
            }
            x = currentPosition[0];
        }
        stopAllMotors();
        return true;
    }
    public void moveTester(){
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setPower(.4);
        op.sleep(1000);
        motorRightBack.setPower(0);
        op.sleep(1000);
        motorRightFront.setPower(.4);
        op.sleep(1000);
        motorRightFront.setPower(0);
        op.sleep(1000);
        motorLeftBack.setPower(.4);
        op.sleep(1000);
        motorLeftBack.setPower(0);
        op.sleep(1000);
        motorLeftFront.setPower(.4);
        op.sleep(1000);
        motorLeftFront.setPower(0);
        op.sleep(1000);
    }
    public void goToPosition(int direction, double y, double x, double a, double power) { //1 for forward, 0 for backward
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        track();
        double[] currentPosition = track();
        double[] startposition = currentPosition;
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double xCorrection=0;
        double yCorrection=0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        double truestartpower=power;
        double powerconst=1;
        double targetspeed = 40*power;
        target_position[0] = x;
        target_position[1] = y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double startDifference=difference;
        double angleConst=0;
        double angleInRadians =0;// atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        double p=.2; double pd = .2; double D=.02; double I =0;
        double xAngle, yAngle;
        double yInt=0,xInt=0,pxError=0,pyError=0,pposxError=0,pposyError=0;
        while ((abs(difference) >= 2.5)) {
            currentPosition = track();
            op.telemetry.addData("distance", difference);
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            if(x==0){
                x=0.0001;
            }
            if(y==0){
                y=0.0001;
            }
            double mpconst=y/x;
            difference-=5;
            double[] tarcurpos={startposition[0]+(target_position[0]-startposition[0])*((startDifference-difference)/startDifference),startposition[1]+(target_position[1]-startposition[1])*(1-(difference)/startDifference)};
            if(difference<0){
                tarcurpos=target_position;
            }
            difference+=5;
            if(xVelocity==0){
                xVelocity=0.0001;
            }
            target_position[2]=atan2(x,y)*180/PI-(direction-1)*180;
            target_position[2]%=360;
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error >= 180) {
                target_position[2] += 360;
            }

            if (error <= -180) {
                target_position[2] -= 360;
            }

            if(mpconst==1){
                mpconst=1.001;
            }
            if(target_position[2]==0){
                target_position[2]=0.00001;
            }
            if(Double.isNaN(target_position[2])){
                target_position[2] = 9999999;
            }
            double xError=xVelocity/abs(xVelocity)*(sqrt(pow(startpower*40,2)/abs((1-pow(mpconst,2))))-abs(xVelocity));
            double posxError=0;//tarcurpos[0]-currentPosition[0];
            double yError=(xError+xVelocity)*mpconst-yVelocity;
            double posyError=0;//(tarcurpos[1]-currentPosition[1]);
            if(difference<5*startpower){
                powerconst=max(min(startpower, startpower*difference/5),0.3/startpower);
                xError = xVelocity/abs(xVelocity)*((pow(difference,2)/4) / abs(1 - pow(mpconst, 2)) - abs(xVelocity));
                yError = (xError + xVelocity) * mpconst - yVelocity;
                targetspeed = pow(difference,2)/4;
            }
//            yInt += (yError + posyError) * differtime;
//            xInt += (xError + posxError) * differtime;
            xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError ) / differtime;
            yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError ) / differtime;
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            if(x==0){
                x=0.0001;
            }
            if(y==0){
                y=0.0001;
            }
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
            angleConst = currentPosition[2] * PI / 180;
            double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
            double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
            if(xCorrection==0){
                xCorrection=0.001;
            }
            if(yCorrection==0){
                yCorrection=0.001;
            }
            if(targetspeed==0){
                targetspeed=0.01;
            }
            target_position[2] = (atan2(x*targetspeed+xCorrection,y*targetspeed+yCorrection)*180/PI)+180*(direction-1);
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
            //error+=(angleInCorrection-currentPosition[2])/2;
            double targetaVelocity = (error)*2;
            anglecorrection=(error*2+(targetaVelocity-aVelocity)/10)/192;
            if(abs(anglecorrection)*power>0.6){
                anglecorrection/=(abs(anglecorrection)*power)/0.6;
            }
            if(aVelocity>280){
                anglecorrection=0;
            }
            double powernum = pow(E, -10*(tan((abs(error / 12) % 15) * PI / 180)));
            if(powernum==-1){
                powernum=-1.0001;
            }
            if(Double.isNaN(powernum)){
                powernum=99999;
            }
            if(error<180&&error>-180) {
                power = startpower * (3-(1 / (1 + powernum)) * 4);
            }
            else{
                power = startpower * (-2.75+(1 / (1 + powernum)) * 4);
            }
            if(direction==0){
                power*=-1;
            }
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
            op.telemetry.addData("ytarget", target_position[1]);
            op.telemetry.addData("xtarget", target_position[0]);
//            op.telemetry.addData("yCorrection", yCorrection);
//            op.telemetry.addData("xCorrection",xCorrection);
//            op.telemetry.addData("p",p);
//            op.telemetry.addData("I",I);
//            op.telemetry.addData("D",D);
//            op.telemetry.addData("xInt",xInt);
//            op.telemetry.addData("yInt",yInt);
//            op.telemetry.addData("yError", yError);
//            op.telemey.adtrdData("xError",xError);
//            op.telemetry.addData("yError", posyError);
//            op.telemetry.addData("xError",posxError);
            op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
            op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
            op.telemetry.addData("power",power);
            op.telemetry.addData("powerconst",powerconst);
            motorRightBack.setPower((powerconst*power   + anglecorrection));
            motorRightFront.setPower((powerconst*power   + anglecorrection));
            motorLeftBack.setPower((powerconst*power   - anglecorrection));
            motorLeftFront.setPower((powerconst*power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);
            pxError=xError;
            pyError=yError;
            pposxError=posxError;
            pposyError=posyError;

            difference = abs(sqrt(x * x + y * y));
            x = currentPosition[0];
            y = currentPosition[1];
        }
        turnInPlace(a,truestartpower);
        stopAllMotors();
    }

    public void goToPositionWithoutStop(int direction, double y, double x, double power) { //1 for forward, 0 for backward
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        track();
        double[] currentPosition = track();
        double[] startposition = currentPosition;
        double[] target_position = {x, y, atan2(x,y)*180/PI-(direction-1)*180};
        double anglecorrection = 0;
        double xCorrection=0;
        double yCorrection=0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        double truestartpower=power;
        double powerconst=1;
        double targetspeed = 30*power;
        target_position[0] = x;
        target_position[1] = y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double startDifference=difference;
        double angleConst=0;
        double angleInRadians =0;// atan2(x, y) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        double p=0.0001,I=0.0000,D=0.0000;
        double xAngle, yAngle;
        double yInt=0,xInt=0,pxError=0,pyError=0,pposxError=0,pposyError=0;
        double runtime=0;
        while ((abs(difference) >= 2.5)) {
            currentPosition = track();
            runtime+=differtime;
            if(runtime>4){
                op.telemetry.addData("DINK DONK TIME GONE", difference);
                break;
            }
            op.telemetry.addData("distance", difference);
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            if(x==0){
                x=0.0001;
            }
            if(y==0){
                y=0.0001;
            }
            double totaldis=x+y;
            double mpconst=y/x;
            difference-=5;
            double[] tarcurpos={startposition[0]+(target_position[0]-startposition[0])*((startDifference-difference)/startDifference),startposition[1]+(target_position[1]-startposition[1])*(1-(difference)/startDifference)};
            if(difference<0){
                tarcurpos=target_position;
            }
            difference+=5;
            if(xVelocity==0){
                xVelocity=0.0001;
            }
            target_position[2]=atan2(x,y)*180/PI-(direction-1)*180;
            target_position[2]%=360;
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error >= 180) {
                target_position[2] += 360;
            }

            if (error <= -180) {
                target_position[2] -= 360;
            }
            op.telemetry.addData("angletarget", target_position[2]);

            if(mpconst==1){
                mpconst=1.001;
            }
            if(target_position[2]==0){
                target_position[2]=0.00001;
            }
            if(Double.isNaN(target_position[2])){
                target_position[2] = 9999999;
            }
            double xError=xVelocity/abs(xVelocity)*(sqrt(pow(startpower*30,2)/abs((1-pow(mpconst,2))))-abs(xVelocity));
            double posxError=tarcurpos[0]-currentPosition[0];
            double yError=(xError+xVelocity)*mpconst-yVelocity;
            double posyError=(tarcurpos[1]-currentPosition[1]);
            if(difference<10*startpower){
                powerconst=max(min(startpower, startpower*difference/10),0.3);
                xError = xVelocity/abs(xVelocity)*((pow(difference,2)/4) / abs(1 - pow(mpconst, 2)) - abs(xVelocity));
                yError = (xError + xVelocity) * mpconst - yVelocity;
                targetspeed = pow(difference,2)/4;
            }
//            yInt += (yError + posyError) * differtime;
//            xInt += (xError + posxError) * differtime;
            xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError ) / differtime;
            yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError ) / differtime;
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            if(x==0){
                x=0.0001;
            }
            if(y==0){
                y=0.0001;
            }
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
            angleConst = currentPosition[2] * PI / 180;
            double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
            double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
            if(xCorrection==0){
                xCorrection=0.001;
            }
            if(yCorrection==0){
                yCorrection=0.001;
            }
            if(targetspeed==0){
                targetspeed=0.01;
            }
            target_position[2] = (atan2(x*targetspeed+xCorrection,y*targetspeed+yCorrection)*180/PI)+180*(direction-1);
            error = currentPosition[2] - target_position[2];
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
            //error+=(angleInCorrection-currentPosition[2])/2;
            double targetaVelocity = (error)*2;
            anglecorrection = ((targetaVelocity + aVelocity)*0.1 + error * 2) / 192;
            if(abs(anglecorrection)>0.6){
                anglecorrection=abs(anglecorrection)/anglecorrection*0.6;
            }
            double powernum = pow(E, -10*(tan((abs(error / 12) % 15) * PI / 180)));
            if(powernum==-1){
                powernum=-1.0001;
            }
            if(Double.isNaN(powernum)){
                powernum=99999;
            }
            if(error<180&&error>-180) {
                power = startpower * (3-(1 / (1 + powernum)) * 4);
            }
            else{
                power = startpower * (-2.75+(1 / (1 + powernum)) * 4);
            }
            if(direction==0){
                power*=-1;
            }
            op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
//            op.telemetry.addData("ytarget", target_position[1]);
//            op.telemetry.addData("xtarget", target_position[0]);
//            op.telemetry.addData("differtime", differtime);
//            op.telemetry.addData("yCorrection", yCorrection);
//            op.telemetry.addData("xCorrection",xCorrection);
//            op.telemetry.addData("p",p);
//            op.telemetry.addData("I",I);
//            op.telemetry.addData("D",D);
//            op.telemetry.addData("xInt",xInt);
//            op.telemetry.addData("yInt",yInt);
//            op.telemetry.addData("yError", yError);
//            op.telemetry.addData("xError",xError);
//            op.telemetry.addData("yError", posyError);
//            op.telemetry.addData("xError",posxError);
//            op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
            op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//            op.telemetry.addData("power",power);
            op.telemetry.addData("targetX",target_position[0]);
            op.telemetry.addData("targetY",target_position[1]);
            op.telemetry.addData("targetA",target_position[2]);
            op.telemetry.addData("x",x);
            op.telemetry.addData("y",y);
            op.telemetry.addData("currentX",currentPosition[0]);
            op.telemetry.addData("currentY",currentPosition[1]);
            op.telemetry.addData("currentA",currentPosition[2]);
            motorRightBack.setPower((powerconst*power   + anglecorrection));
            motorRightFront.setPower((powerconst*power   + anglecorrection));
            motorLeftBack.setPower((powerconst*power   - anglecorrection));
            motorLeftFront.setPower((powerconst*power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);
            pxError=xError;
            pyError=yError;
            pposxError=posxError;
            pposyError=posyError;

            difference = abs(sqrt(x * x + y * y));
            x = currentPosition[0];
            y = currentPosition[1];
        }

        stopAllMotors();
    }

    //direction=1 for robot start angle, direction = 0 for backwards
    public void tripleSplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power, double targetAnglu) {
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
        double[] startPosition = currentPosition;
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
        double p=0.01,I=0.0000,D=0.00057;
        double mpyVelocity = 0;
        double[] tarcurpos = {0,0,0};
        runtime.reset();
        double pxError=0,pyError=0, pposxError=0,pposyError=0, x=0, y=0 ,xError=0, posxError=0, yError=0, posyError=0, xCorrection=0, yCorrection=0, approxDifference=0, t=0,yInt=0,xInt=0,angleConst;
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
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/6)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i+1][0]) + (-point[i+1][0] + point[i+2][0]) * t + (2 * point[i+1][0] - 5 * point[i+1][0] + 4 * point[i+2][0] - point[i+3][0]) * pow(t, 2) +
                        (-point[i+1][0] + 3 * point[i+1][0] - 3 * point[i+2][0] + point[i+3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i+1][1]) + (-point[i+1][1] + point[i+2][1]) * t + (2 * point[i+1][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * pow(t, 2) +
                        (-point[i+1][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]-point[i+1][1]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]-point[i+1][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * ( + ( + point[i + 2][1]-point[i+1][1]) + 2 * ( 2*point[i+1][1]- 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * ( -point[i+1][1]+ 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                target_position[2] = startPosition[2]+(targetAnglu-startPosition[2])*t/3;
                error = currentPosition[2]-target_position[2];
                error %= 360;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                xError=sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2)))-xVelocity;
                posxError=tarcurpos[0]-currentPosition[0];
                yError=(xError+xVelocity)*mpconst-yVelocity;
                posyError=(tarcurpos[1]-currentPosition[1]);
                yInt+=(yError+posyError)*differtime;
                xInt+=(xError+posxError)*differtime;
                xCorrection=p*(xError+posxError)+I*xInt+D*(xError+posxError-pposxError-pxError)/differtime;
                xCorrection*=-1;
                yCorrection=p*(yError+posyError)+I*yInt+D*(yError+posyError-pposyError-pyError)/differtime;
                angleConst=currentPosition[2]*PI/180;
                double angleInCorrection = atan2(yCorrection, xCorrection)-(angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection+PI /4),sin(angleInCorrection-PI/4),sqrt(pow(yCorrection,2)+pow(xCorrection,2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-error);
                anglecorrection = power*((-targetaVelocity+aVelocity)+error*3)/135;

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
//                op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
//                op.telemetry.addData("ytarget", target_position[1]);
//                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
                motorRightBack.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
                pxError=xError;
                pyError=yError;
                pposxError=posxError;
                pposyError=posyError;
            }
        }
        for (int i =0; i < 1; i++) {
            startPosition=currentPosition;
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            t=0;

            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/6)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i+1][0]) + (-point[i+0][0] + point[i+2][0]) * t + (2 * point[i+0][0] - 5 * point[i+1][0] + 4 * point[i+2][0] - point[i+3][0]) * pow(t, 2) +
                        (-point[i+0][0] + 3 * point[i+1][0] - 3 * point[i+2][0] + point[i+3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i+1][1]) + (-point[i+0][1] + point[i+2][1]) * t + (2 * point[i+0][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * pow(t, 2) +
                        (-point[i+0][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]-point[i+0][1]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]-point[i+0][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * ( + ( + point[i + 2][1]-point[i+0][1]) + 2 * ( 2*point[i][1]- 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * ( -point[i][1]+ 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                target_position[2] = startPosition[2]+(targetAnglu-startPosition[2])*(t+1)/3;
                error = currentPosition[2]-target_position[2];
                error %= 360;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                xError=sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2)))-xVelocity;
                posxError=tarcurpos[0]-currentPosition[0];
                yError=(xError+xVelocity)*mpconst-yVelocity;
                posyError=(tarcurpos[1]-currentPosition[1]);
                yInt+=(yError+posyError)*differtime;
                xInt+=(xError+posxError)*differtime;
                xCorrection=p*(xError+posxError)+I*xInt+D*(xError+posxError-pposxError-pxError)/differtime;
                xCorrection*=-1;
                yCorrection=p*(yError+posyError)+I*yInt+D*(yError+posyError-pposyError-pyError)/differtime;
                angleConst=currentPosition[2]*PI/180;
                double angleInCorrection = atan2(yCorrection, xCorrection)-(angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection+PI /4),sin(angleInCorrection-PI/4),sqrt(pow(yCorrection,2)+pow(xCorrection,2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-error);
                anglecorrection = power*((-targetaVelocity+aVelocity)+error*3)/135;

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
//                op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
//                op.telemetry.addData("ytarget", target_position[1]);
//                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
                motorRightBack.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
                pxError=xError;
                pyError=yError;
                pposxError=posxError;
                pposyError=posyError;
            }
        }
        for (int i =1; i < 2; i++) {
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            t=0;
            boolean td=true, approximated=false;
            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>t&&td){
                    t = (oneDistance+Velocity/6)/(oneDistance+twoDistance);
                }
                if(t>1){
                    t=1;td=false;
                }
                if(!td){
                    t=1;
                    target_position[0]=x3;
                    target_position[1]=y3;
                }
                target_position[0] = 0.5 * ((2 * point[i+1][0]) + (-point[i+0][0] + point[i+2][0]) * t + (2 * point[i+0][0] - 5 * point[i+1][0] + 4 * point[i+2][0] - point[i+3][0]) * pow(t, 2) +
                        (-point[i+0][0] + 3 * point[i+1][0] - 3 * point[i+2][0] + point[i+3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i+1][1]) + (-point[i+0][1] + point[i+2][1]) * t + (2 * point[i+0][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * pow(t, 2) +
                        (-point[i+0][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 3));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]-point[i+0][1]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]-point[i+0][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * ( + ( + point[i + 2][1]-point[i+0][1]) + 2 * ( 2*point[i][1]- 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * ( -point[i][1]+ 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                mpconst=target_position[2];
                target_position[2]=atan(1/target_position[2]) + (direction-1) * PI;
                target_position[2] = startPosition[2]+(targetAnglu-startPosition[2])*(t+1)/3;
                error = currentPosition[2]-target_position[2];
                error %= 360;
                approxDifference = 0;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                xError = sqrt(pow((power) * power * 30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                posxError = tarcurpos[0] - currentPosition[0];
                yError = (xError + xVelocity) * mpconst - yVelocity;
                posyError = (tarcurpos[1] - currentPosition[1]);
                if(difference<20){
                    double tdiff= (1-t)/20;
                    double[] dummyPosition = {0,0};
                    double[] dummyPositionTwo = {currentPosition[0],currentPosition[1]};
                    if(!approximated) {
                        for (double j = t; j < 1 + tdiff; j += tdiff) {
                            dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * j + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0]) * pow(j, 2) +
                                    (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0]) * pow(j, 3));

                            dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * j + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1]) * pow(j, 2) +
                                    (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1]) * pow(j, 3));
                            approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                            dummyPositionTwo = dummyPosition;
                        }
                        //approximated=true;
                    }
                    else{
                        approxDifference-=Velocity*differtime;
                    }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                    //power=startpower;
                    //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                    if(approxDifference<5*pow(startpower,2)){
                        power=max(min(startpower, startpower*approxDifference/10/power),0.4);
                        xError = sqrt((pow(approxDifference,2)/1*power*power) / (1 + pow(mpconst, 2))) - xVelocity;
                        yError = (xError + xVelocity) * mpconst - yVelocity;

                    }
                    else{
                        power=startpower;
                    }
                }
                yInt+=(yError+posyError)*differtime;
                xInt+=(xError+posxError)*differtime;
                xCorrection=p*(xError+posxError)+I*xInt+D*(xError+posxError-pposxError-pxError)/differtime;
                xCorrection*=-1;
                yCorrection=p*(yError+posyError)+I*yInt+D*(yError+posyError-pposyError-pyError)/differtime;
                angleConst=currentPosition[2]*PI/180;
                double angleInCorrection = atan2(yCorrection, xCorrection)-(angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection+PI /4),sin(angleInCorrection-PI/4),sqrt(pow(yCorrection,2)+pow(xCorrection,2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity=(-error);
                anglecorrection = power*((-targetaVelocity+aVelocity)+error*3)/135;

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
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                motorRightBack.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0]+angleCorrectPower[0]*angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1]+angleCorrectPower[1]*angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
                pxError=xError;
                pyError=yError;
                pposxError=posxError;
                pposyError=posyError;
            }
        }
        stopAllMotors();
    }
    public void partOfPolySplineToPosition(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power, double targetAnglu) {
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
        point[4][0] = x3+x3-x2;
        point[4][1] = y3+y3-y2;
        double[] currentPosition = track();
        double[] startPosition = currentPosition;
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
        double mpconst = 0;
        double p=0.01,I=0.0000,D=0.00057;
        double mpyVelocity = 0;
        double[] tarcurpos = {0,0,0};
        runtime.reset();
        double pxError=0,pyError=0, pposxError=0,pposyError=0, x=0, y=0 ,xError=0, posxError=0, yError=0, posyError=0, xCorrection=0, yCorrection=0, approxDifference=0, t=0,yInt=0,xInt=0,angleConst;
        if (start) {
            for (int i = -1; i < 0; i++) {
                double timedis = sqrt(pow(point[i + 2][0] - point[i + 1][0], 2) + pow(point[i + 2][1] - point[i + 1][1], 2));
                startPosition = currentPosition;
                axisa = atan2(-point[i + 2][1] + point[i + 1][1], point[i + 2][0] - point[i + 1][0]);
//            double looptime=0;
//            double lasteTime=0;
//            double thisetime=0;
                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    //thisetime=runtime.seconds();
                    //looptime=thisetime-lasteTime;
                    //lasteTime=thisetime;

                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > 1) {
                        break;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 1][0] + point[i + 2][0]) * t + (2 * point[i + 1][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 1][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 1][1] + point[i + 2][1]) * t + (2 * point[i + 1][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 1][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 1][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 1][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][1] - point[i + 1][1]) + 2 * (2 * point[i + 1][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i + 1][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    mpconst = target_position[2];
                    target_position[2] = atan(1 / target_position[2]) + (direction - 1) * PI;
                    target_position[2] = startPosition[2] + (targetAnglu - startPosition[2]) * t;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    xError = sqrt(pow((power) *30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    angleConst = currentPosition[2] * PI / 180;
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    angleInRadians = atan2(y, -x) - (angleConst);
                    anglePower[0] = sin(angleInRadians + PI / 4);
                    anglePower[1] = sin(angleInRadians - PI / 4);
                    error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 2) / 135;

                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);
                    }

                    op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                    op.telemetry.addData("ytarget", target_position[1]);
                    op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                    motorRightBack.setPower((power   + anglecorrection));
                    motorRightFront.setPower((power   + anglecorrection));
                    motorLeftBack.setPower((power   - anglecorrection));
                    motorLeftFront.setPower((power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
            }
        }
        if(!start&&!end) {
            for (int i = 0; i < 1; i++) {
                startPosition = currentPosition;
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;

                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > 1) {
                        break;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    mpconst = target_position[2];
                    target_position[2] = atan(1 / target_position[2]) + (direction - 1) * PI;
                    target_position[2] = startPosition[2] + (targetAnglu - startPosition[2]) * t;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    xError = sqrt(pow((power) * power * 30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    angleConst = currentPosition[2] * PI / 180;
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    angleInRadians = atan2(y, -x) - (angleConst);
                    anglePower[0] = sin(angleInRadians + PI / 4);
                    anglePower[1] = sin(angleInRadians - PI / 4);
                    error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 2) / 135;

                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);
                    }

                    op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                    op.telemetry.addData("ytarget", target_position[1]);
                    op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                    motorRightBack.setPower((power   + anglecorrection));
                    motorRightFront.setPower((power   + anglecorrection));
                    motorLeftBack.setPower((power   - anglecorrection));
                    motorLeftFront.setPower((power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
            }
        }
        if (end) {
            for (int i = 1; i < 2; i++) {
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;
                boolean td = true, approximated = false;
                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t && td) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if (t > 1) {
                        t = 1;
                        td = false;
                    }
                    if (!td) {
                        t = 1;
                        target_position[0] = x3;
                        target_position[1] = y3;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    mpconst = target_position[2];
                    target_position[2] = atan(1 / target_position[2]) + (direction - 1) * PI;
                    target_position[2] = startPosition[2] + (targetAnglu - startPosition[2]) * t;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    approxDifference = 0;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    xError = sqrt(pow((power) * power *30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    if (difference < 20) {
                        double tdiff = (1 - t) / 20;
                        double[] dummyPosition = {0, 0};
                        double[] dummyPositionTwo = {currentPosition[0], currentPosition[1]};
                        if (!approximated) {
                            for (double j = t; j < 1 + tdiff; j += tdiff) {
                                dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * j + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0]) * pow(j, 2) +
                                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0]) * pow(j, 3));

                                dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * j + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1]) * pow(j, 2) +
                                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1]) * pow(j, 3));
                                approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                                dummyPositionTwo = dummyPosition;
                            }
                            //approximated=true;
                        } else {
                            approxDifference -= Velocity * differtime;
                        }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                        //power=startpower;
                        //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                        if (approxDifference < 5 * pow(startpower, 2)) {
                            power = max(min(startpower, startpower * approxDifference / 10 / power), 0.4);
                            xError = sqrt((pow(approxDifference, 2) / 1 * power * power) / (1 + pow(mpconst, 2))) - xVelocity;
                            yError = (xError + xVelocity) * mpconst - yVelocity;

                        } else {
                            power = startpower;
                        }
                    }
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    angleConst = currentPosition[2] * PI / 180;
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    angleInRadians = atan2(y, -x) - (angleConst);
                    anglePower[0] = sin(angleInRadians + PI / 4);
                    anglePower[1] = sin(angleInRadians - PI / 4);
                    error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 2) / 135;

                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);
                    }

                    op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                    op.telemetry.addData("ytarget", target_position[1]);
                    op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                    motorRightBack.setPower((power   + anglecorrection));
                    motorRightFront.setPower((power   + anglecorrection));
                    motorLeftBack.setPower((power   - anglecorrection));
                    motorLeftFront.setPower((power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
                stopAllMotors();
            }
        }
        if(start&&end){
            for (int i = 0; i < 1; i++) {
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;
                boolean td = true, approximated = false;
                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t && td) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if (t > 1) {
                        t = 1;
                        td = false;
                    }
                    if (!td) {
                        t = 1;
                        target_position[0] = x3;
                        target_position[1] = y3;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    mpconst = target_position[2];
                    target_position[2] = atan(1 / target_position[2]) + (direction - 1) * PI;
                    target_position[2] = startPosition[2] + (targetAnglu - startPosition[2]) * (t);
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    approxDifference = 0;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    xError = sqrt(pow((power) * power *30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    if (difference < 20) {
                        double tdiff = (1 - t) / 20;
                        double[] dummyPosition = {0, 0};
                        double[] dummyPositionTwo = {currentPosition[0], currentPosition[1]};
                        if (!approximated) {
                            for (double j = t; j < 1 + tdiff; j += tdiff) {
                                dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * j + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0]) * pow(j, 2) +
                                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0]) * pow(j, 3));

                                dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * j + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1]) * pow(j, 2) +
                                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1]) * pow(j, 3));
                                approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                                dummyPositionTwo = dummyPosition;
                            }
                            //approximated=true;
                        } else {
                            approxDifference -= Velocity * differtime;
                        }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                        //power=startpower;
                        //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                        if (approxDifference < 5 * pow(startpower, 2)) {
                            power = max(min(startpower, startpower * approxDifference / 10 / power), 0.4);
                            xError = sqrt((pow(approxDifference, 2) / 1 * power * power) / (1 + pow(mpconst, 2))) - xVelocity;
                            yError = (xError + xVelocity) * mpconst - yVelocity;

                        } else {
                            power = startpower;
                        }
                    }
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    angleConst = currentPosition[2] * PI / 180;
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    angleInRadians = atan2(y, -x) - (angleConst);
                    anglePower[0] = sin(angleInRadians + PI / 4);
                    anglePower[1] = sin(angleInRadians - PI / 4);
                    error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 2) / 135;

                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);
                    }

                    op.telemetry.addData("difference", difference);
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                    op.telemetry.addData("ytarget", target_position[1]);
                    op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                    motorRightBack.setPower((power   + anglecorrection));
                    motorRightFront.setPower((power   + anglecorrection));
                    motorLeftBack.setPower((power   - anglecorrection));
                    motorLeftFront.setPower((power   - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
                stopAllMotors();
            }
        }
    }
    public void partOfPolySplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power) {
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
        point[4][0] = x3+(x3-x2)*pow(abs((x3-x2)/(y3-y2)),1);
        point[4][1] = y3+(y3-y2)*pow(abs((y3-y2)/(x3-x2)),1);
        double[] currentPosition = track();
        double[] startPosition = currentPosition;
        double[] target_position = {0, 0, 0};
        double anglecorrection = 0;
        double maxpower = 0.2;
        double time = op.getRuntime();
        double difftime = 0;
        double diffpos = 0;
        double sped = 0;
        double stoptime = 0;
        double tt = 0;


        double axisa = atan2(x2 - x1, y2 - y1);
        target_position[0] = x3;
        target_position[1] = y3;
        target_position[2] =0;
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(point[2][0] - point[1][0], point[2][1] - point[1][1]) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        double startpower = power;
        double error = 0;
        double max = 0.15;
        double powerconst=1, xAngle=0, yAngle=0, targetspeed=startpower*30;
        double mpconst = 0;
        double p=0.0001,I=0.0000,D=0.000;
        double mpyVelocity = 0;
        double[] tarcurpos = {0,0,0};
        runtime.reset();
        double pxError=0,pyError=0, pposxError=0,pposyError=0, x=0, y=0 ,xError=0, posxError=0, yError=0, posyError=0, xCorrection=0, yCorrection=0, approxDifference=0, t=0,yInt=0,xInt=0,angleConst;
        if (start&&!end) {
            for (int i = -1; i < 0; i++) {
                double timedis = sqrt(pow(point[i + 2][0] - point[i + 1][0], 2) + pow(point[i + 2][1] - point[i + 1][1], 2));
                startPosition = currentPosition;
                axisa = atan2(-point[i + 2][1] + point[i + 1][1], point[i + 2][0] - point[i + 1][0]);
//            double looptime=0;
//            double lasteTime=0;
//            double thisetime=0;
                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    //thisetime=runtime.seconds();
                    //looptime=thisetime-lasteTime;
                    //lasteTime=thisetime;

                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 2+1) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > 1) {
                        break;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 1][0] + point[i + 2][0]) * t + (2 * point[i + 1][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 1][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 1][1] + point[i + 2][1]) * t + (2 * point[i + 1][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 1][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][0]) + 2 * ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                            3 * (+ 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] ) + 2 * ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    if(target_position[2]==0){
                        target_position[2]=0.00001;
                    }
                    if(Double.isNaN(target_position[2])){
                        target_position[2] = 9999999;
                    }
                    mpconst=1/target_position[2];
//                    if(target_position[2]!=0) {
//                        mpconst = 1 / target_position[2];
//                    }
//                    else{
//                        target_position[2]=0.001;
//                        mpconst = 1000;
//                    }
                    target_position[2]=(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 1][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 1][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if(mpconst==1){
                        mpconst=1.001;
                    }
                    xError = sqrt(pow(startpower *30, 2) / abs(1 - pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    posyError = (tarcurpos[1] - currentPosition[1]);

//            yInt += (yError + posyError) * differtime;
//            xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError ) / differtime;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError ) / differtime;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    if(x==0){
                        x=0.0001;
                    }
                    if(y==0){
                        y=0.0001;
                    }
                    double totaldis=sqrt(x*x+y*y);
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
                    target_position[2] -= (atan(yCorrection/xCorrection)*180/PI/90-target_position[2])*correctionMag/targetspeed;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    //error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = startpower*((-targetaVelocity + aVelocity) + error * 2) / 135;
                    if(error<180&&error>-180) {
                        power = startpower * (1+(1 / (1 + pow(E, -10 * (tan((-abs(error / 2) % 90) * PI / 180))))) * 2);
                    }
                    else{
                        power=startpower * (1+(1 / (1 + pow(E, -10 * (tan((abs(error / 2)) * PI / 180))))) * 2);
                    }
                    if(direction==0){
                        power*=-1;
                    }

                    op.telemetry.addData("difference", difference);
                    op.telemetry.addData("angleTarget", target_position[2]);
                    op.telemetry.addData("error", error);


                    motorRightBack.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorRightFront.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorLeftBack.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));
                    motorLeftFront.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));
                    error = currentPosition[2]-target_position[2];
                    error %= 360;

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
            }
        }
        if(!start&&!end) {
            for (int i = 0; i < 1; i++) {
                startPosition = currentPosition;
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;

                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > 1) {
                        break;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    target_position[2] = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                            3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    if(target_position[2]==0){
                        target_position[2]=0.00001;
                    }
                    if(Double.isNaN(target_position[2])){
                        target_position[2] = 9999999;
                    }
                    mpconst=1/target_position[2];
//                    if(target_position[2]!=0) {
//                        mpconst = 1 / target_position[2];
//                    }
//                    else{
//                        target_position[2]=0.001;
//                        mpconst = 1000;
//                    }
                    target_position[2]=(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                    error = currentPosition[2]-target_position[2];
                    error %= 360;
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    if(mpconst==1){
                        mpconst=1.001;
                    }
                    xError = sqrt(pow(startpower *30, 2) / abs(1 - pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    if(x==0){
                        x=0.0001;
                    }
                    if(y==0){
                        y=0.0001;
                    }
                    double totaldis=sqrt(x*x+y*y);
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
                    target_position[2] -= (atan(yCorrection/xCorrection)*180/PI/90-target_position[2])*correctionMag/targetspeed;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    //error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = startpower*((-targetaVelocity + aVelocity) + error * 2) / 135;
                    if(error<180&&error>-180) {
                        power = startpower * (1+(1 / (1 + pow(E, -10 * (tan((-abs(error / 2) % 90) * PI / 180))))) * 2);
                    }
                    else{
                        power=startpower * (1+(1 / (1 + pow(E, -10 * (tan((abs(error / 2)) * PI / 180))))) * 2);
                    }


                    op.telemetry.addData("difference", difference);
                    op.telemetry.addData("angleTarget", target_position[2]);
                    op.telemetry.addData("error", error);


                    motorRightBack.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorRightFront.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorLeftBack.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));
                    motorLeftFront.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));

                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
            }
        }
        if (end&&!start) {
            for (int i = 1; i < 2; i++) {
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;
                boolean td = true, approximated = false;
                while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t && td) {
                        t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                    }
                    if (t > 1) {
                        t = 1;
                        td = false;
                    }
                    if (!td) {
                        t = 1;
                        target_position[0] = x2;
                        target_position[1] = y2;
                    }
                    if(td) {
                        target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                                (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                        target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                                (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    }
                    target_position[2] = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                            3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    if(target_position[2]==0){
                        target_position[2]=0.00001;
                    }
                    if(Double.isNaN(target_position[2])){
                        target_position[2] = 9999999;
                    }
                    mpconst=1/target_position[2];
//                    if(target_position[2]!=0) {
//                        mpconst = 1 / target_position[2];
//                    }
//                    else{
//                        target_position[2]=0.001;
//                        mpconst = 1000;
//                    }
                    target_position[2]=(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                    error = currentPosition[2]-target_position[2];
                    error %= 360;
                    if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                        t = (oneDistance+1
                        ) / (oneDistance + twoDistance);
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                    approxDifference = 0;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    if(mpconst==1){
                        mpconst=1.001;
                    }
                    xError = sqrt(pow(startpower *30, 2) / abs(1 - pow(mpconst, 2))) - xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = (xError + xVelocity) * mpconst - yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    if (difference < 20) {
                        double tdiff = (1 - t) / 20;
                        double[] dummyPosition = {0, 0};
                        double[] dummyPositionTwo = {currentPosition[0], currentPosition[1]};
                        if (!approximated) {
                            for (double j = t; j < 1 + tdiff; j += tdiff) {
                                dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                                dummyPosition[1] = target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                                approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                                dummyPositionTwo = dummyPosition;
                            }
                            //approximated=true;
                        } else {
                            approxDifference -= Velocity * differtime;
                        }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                        //power=startpower;
                        //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                        if (approxDifference < 5 * pow(startpower, 2)) {
                            powerconst = max(min(startpower, startpower * approxDifference / 10 / power), 0.4);
                            xError = sqrt((pow(approxDifference, 2) / 1 * power * power) / (1 - pow(mpconst, 2))) - xVelocity;
                            yError = (xError + xVelocity) * mpconst - yVelocity;
                            targetspeed = pow(startpower,1/3)*pow(difference,2)/4;
                        } else {
                            power = startpower;
                        }
                    }
                    yInt += (yError + posyError) * differtime;
                    xInt += (xError + posxError) * differtime;
                    xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                    xCorrection *= -1;
                    yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                    x = target_position[0] - currentPosition[0];
                    y = target_position[1] - currentPosition[1];
                    if(x==0){
                        x=0.0001;
                    }
                    if(y==0){
                        y=0.0001;
                    }
                    double totaldis=sqrt(x*x+y*y);
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
                    target_position[2] -= (atan(yCorrection/xCorrection)*180/PI/90-target_position[2])*correctionMag/targetspeed;
                    error = currentPosition[2] - target_position[2];
                    error %= 360;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    //error+=(angleInCorrection-currentPosition[2])/2;
                    double targetaVelocity = (-error)*2;
                    anglecorrection = startpower*((-targetaVelocity + aVelocity) + error * 2) / 90;
                    if(error<180&&error>-180) {
                        power = startpower * (1+(1 / (1 + pow(E, -10 * (tan((-abs(error / 2) % 90) * PI / 180))))) * 2);
                    }
                    else{
                        power=startpower * (1+(1 / (1 + pow(E, -10 * (tan((abs(error / 2)) * PI / 180))))) * 2);
                    }


                    op.telemetry.addData("difference", difference);
                    op.telemetry.addData("angleTarget", target_position[2]);
                    op.telemetry.addData("error", error);


                    motorRightBack.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorRightFront.setPower(((powerconst+correctionMag/30)*power   + anglecorrection));
                    motorLeftBack.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));
                    motorLeftFront.setPower(((powerconst+correctionMag/30)*power   - anglecorrection));
                    difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
            }
        }
        if(start&&end){
            double lastAngle=track()[2], anglediff=0, targetaVelocity=0;
            p=.2; double pd = .2; D=.02;
            for (int i = 0; i < 1; i++) {
                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                t = 0;
                tt=0;
                double t2=0;
                targetspeed=startpower*30;
                double xDerivative=0, yDerivative=0;
                boolean td = true, approximated = false;
                while (op.opModeIsActive() && (abs(difference) >= 2.5)) {
                    currentPosition = track();
                    double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                    double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                    if ((oneDistance + Velocity/2+1/4) / (oneDistance + twoDistance) > t && td) {
                        t = (oneDistance + Velocity/2+1/4) / (oneDistance + twoDistance);
                        t2=(oneDistance+Velocity/2+1/4)/(oneDistance+twoDistance);
                    }
                    if (t >= 1) {
                        t = 1;
                    }
                    if (!td) {
                        t = 1;
                        td=true;
//                        target_position[0] = x2;
//                        target_position[1] = y2;
//                        xDerivative = target_position[0]-currentPosition[0];
//                        yDerivative = target_position[1]-currentPosition[1];
//                        target_position[2]=atan2(x2,y2)*180/PI-(direction*180);
//                        target_position[2]%=360;
//                        error = currentPosition[2] - target_position[2];
//                        error %= 360;
//                        if (error > 180) {
//                            target_position[2] += 360;
//                        }
//                        if (error < -180) {
//                            target_position[2] -= 360;
//                        }
//                        td=true;
                    }
                    target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t2 + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t2, 2) +
                            (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t2, 3));

                    target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t2 + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t2, 2) +
                            (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t2, 3));
                    xDerivative = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                            3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)));
                    yDerivative = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                    if(yDerivative==0){
                        yDerivative=0.00001;
                    }
                    if(xDerivative==0){
                        xDerivative=0.00001;
                    }
//                    op.telemetry.addData("xTarget", target_position[0]);
//                    op.telemetry.addData("yTarget", target_position[1]);
//                    op.telemetry.addData("t", t);
                    mpconst=yDerivative/xDerivative;
//                    if(target_position[2]!=0) {
//                        mpconst = 1 / target_position[2];
//                    }
//                    else{
//                        target_position[2]=0.001;
//                        mpconst = 1000;
//                    }
                        target_position[2]=atan2(xDerivative,yDerivative)*180/PI-(direction-1)*180;
                        anglediff=target_position[2]-lastAngle;
                        targetaVelocity=-anglediff/differtime;
                        lastAngle = target_position[2];
                        target_position[2]%=360;
                        error = currentPosition[2] - target_position[2];
                        error %= 360;
                        if (error > 180) {
                            target_position[2] += 360;
                        }
                        if (error < -180) {
                            target_position[2] -= 360;
                        }
                    op.telemetry.addData("angleTarget", target_position[2]);
                    if ((oneDistance+1/4) / (oneDistance + twoDistance) > tt) {
                        tt = (oneDistance+1/4) / (oneDistance + twoDistance);
                    }
                    if(tt>1){
                        tt=1;
                    }
                    tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * tt + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(tt, 2) +
                            (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(tt, 3));

                    tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * tt + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(tt, 2) +
                            (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(tt, 3));
                    double txDerivative = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * tt +
                            3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(tt, 2)));
                    double tyDerivative = (0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * tt +
                            3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(tt, 2)));
                    tarcurpos[2]=atan2(txDerivative,tyDerivative)*180/PI-(direction-1)*180;
                    if(tyDerivative==0){
                        tyDerivative=0.00001;
                    }
                    if(txDerivative==0){
                        txDerivative=0.00001;
                    }
                    double mpconst2=tyDerivative/txDerivative;
                    approxDifference = 0;
                    if(mpconst==1){
                        mpconst=1.001;
                    }
                    if(mpconst2==1){
                        mpconst2=1.001;
                    }
                    if(xVelocity==0){
                        xVelocity=0.0001;
                    }
                    if(!td){
                        tarcurpos[0]=x2;
                        tarcurpos[1]=y2;
                    }
                    double targetXVelocity = txDerivative/abs(txDerivative)*((sqrt(pow(startpower *30, 2) / abs(1 - pow(mpconst2, 2)))));
                    double targetYVelocity=targetXVelocity*mpconst2;
                    xError = targetXVelocity-xVelocity;
                    posxError = tarcurpos[0] - currentPosition[0];
                    yError = targetYVelocity-yVelocity;
                    posyError = (tarcurpos[1] - currentPosition[1]);
                    targetXVelocity = xDerivative/abs(xDerivative)*((sqrt(pow(startpower *30, 2) / abs(1 - pow(mpconst, 2)))));
                    targetYVelocity=targetXVelocity*mpconst;

                    /*if (difference < 20) {
                        double tdiff = (1 - tt) / 20;
                        double[] dummyPosition = {0, 0};
                        double[] dummyPositionTwo = {currentPosition[0], currentPosition[1]};
                        if (!approximated) {
                            for (double j = t; j < 1 + tdiff; j += tdiff) {
                                dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                                dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                                approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                                dummyPositionTwo = dummyPosition;
                            }
                            //approximated=true;
                        } else {
                            approxDifference -= Velocity * differtime;
                        }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                        //power=startpower;
                        //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                        if (approxDifference < 5 *startpower) {
                            powerconst = max(min(startpower, startpower * approxDifference / 10 / power), 0.4);
                            targetspeed = pow(startpower,1/3)*pow(difference,2)/4;
                        } else {
                            powerconst = 1;
                        }
                    }*/

//            yInt += (yError + posyError) * differtime;
//            xInt += (xError + posxError) * differtime;
                    xCorrection = pd*xError + p*posxError+ I * xInt + D * (xError + posxError/differtime );
                    yCorrection = pd*yError + p*posyError + I * yInt + D * (yError + posyError / differtime);
                    x =point[i+2][0] - currentPosition[0];
                    y = point[i+2][1] - currentPosition[1];
                    if(x==0){
                        x=0.0001;
                    }
                    if(y==0){
                        y=0.0001;
                    }
                    double totaldis=sqrt(x*x+y*y);
//                        if(difference<10*startpower*startpower){
//                            powerconst=max(min(startpower, startpower*difference/10),0.3);
//                            xError = (pow(difference,2)/4) / (1 + pow(mpconst, 2)) - xVelocity;
//                            yError = (xError + xVelocity) * mpconst - yVelocity;
//                            targetspeed = pow(difference,2)/4;
//                        }
                    angleConst = currentPosition[2] * PI / 180;
                    double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                    double correctionMag =30*sqrt(pow(xCorrection,2)+pow(yCorrection,2));
                    if(targetspeed==0){
                        targetspeed=0.01;
                    }
                    if(xCorrection==0){
                        xCorrection=0.001;
                    }
                    if(yCorrection==0){
                        yCorrection=0.001;
                    }
//                    posxError=0;
//                    posyError=0;
                    target_position[2] = (atan2(targetXVelocity + xCorrection, targetYVelocity + yCorrection) * 180 / PI)-(direction-1)*180;

//                    op.telemetry.addData("xDerivative",targetXVelocity);
//                    op.telemetry.addData("xcorrection",xCorrection);
//                    op.telemetry.addData("yDerivative",targetYVelocity);
//                    op.telemetry.addData("ycorrection",yCorrection);
//                    op.telemetry.addData("acorrection",(atan2(targetXVelocity+xCorrection,targetYVelocity+yCorrection))*180/PI);
                    error = currentPosition[2] - target_position[2];
                    //double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                    error+=(currentPosition[2]-tarcurpos[2]);
                    error %= 360;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    op.telemetry.addData("error", error);
                    double error2=currentPosition[2]-atan2(x,y)*180/PI-(direction-1)*180;
                    error2*=1;
                    error2 %= 360;
                    if (error2 > 180) {
                        error2 -= 360;
                    }
                    if (error2 < -180) {
                        error2 += 360;
                    }
                    double controlconst=(t*t*t);
                    op.telemetry.addData("erro2r", error2);
                    error=controlconst*error2+((1-controlconst)*error);
                    error %= 360;
                    if (error > 180) {
                        error -= 360;
                    }
                    if (error < -180) {
                        error += 360;
                    }
                    targetaVelocity += (error)*2;
                    anglecorrection = ((targetaVelocity + aVelocity)*0.1 + error * 2) / 192;
                    double powernum = pow(E, -10*(tan((abs(error / 12) % 15) * PI / 180)));
                    if(powernum==-1){
                        powernum=-1.0001;
                    }
                    if(Double.isNaN(powernum)){
                        powernum=99999;
                    }
                    if(error<180&&error>-180) {
                        power = startpower * (3-(1 / (1 + powernum)) * 4);
                    }
                    else{
                        power = startpower * (-2.75+(1 / (1 + powernum)) * 4);
                    }
                    if(direction==0){
                        power*=-1;
                    }
                    op.telemetry.addData("t", t);
                    op.telemetry.addData("error", error);
                    op.telemetry.addData("targetXPosition",target_position[0]);
                    op.telemetry.addData("targetYPosition",target_position[1]);

//                    op.telemetry.addData("power", power);
//                    op.telemetry.addData("anglecrrrection", anglecorrection);
                    motorRightBack.setPower(((powerconst)*power   + anglecorrection));
                    motorRightFront.setPower(((powerconst)*power   + anglecorrection));
                    motorLeftBack.setPower(((powerconst)*power   - anglecorrection));
                    motorLeftFront.setPower(((powerconst)*power   - anglecorrection));

                    difference = sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2));
                    pxError = xError;
                    pyError = yError;
                    pposxError = posxError;
                    pposyError = posyError;
                }
                stopAllMotors();
            }
        }
    }
    public void tripleSplineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double power) {
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
        double[] startPosition = currentPosition;
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
        double p=0.01,I=0.0000,D=0.00057;
        double mpyVelocity = 0;
        double[] tarcurpos = {0,0,0};
        runtime.reset();
        double pxError=0,pyError=0, pposxError=0,pposyError=0, x=0, y=0 ,xError=0, posxError=0, yError=0, posyError=0, xCorrection=0, yCorrection=0, approxDifference=0, t=0,yInt=0,xInt=0,angleConst;
        for (int i =-1; i < 0; i++) {
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
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/6)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * (+(+point[i + 2][0]) + 2 * ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                        3 * ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] ) + 2 * ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                mpconst=target_position[2];
                target_position[2]=-(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                error = currentPosition[2]-target_position[2];
                error %= 360;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                if(1-pow(mpconst,2)==0){
                    mpconst=1.001;
                }
                xError=sqrt(pow((power)*power*30,2)/(1-pow(mpconst,2)))-xVelocity;
                posxError=tarcurpos[0]-currentPosition[0];
                yError=(xError+xVelocity)*mpconst-yVelocity;
                posyError=(tarcurpos[1]-currentPosition[1]);
                yInt += (yError + posyError) * differtime;
                xInt += (xError + posxError) * differtime;
                xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                xCorrection *= -1;
                yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                angleConst = currentPosition[2] * PI / 180;
                double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity = (-error);
                anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 3) / 135;

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
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                motorRightBack.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
                pxError=xError;
                pyError=yError;
                pposxError=posxError;
                pposyError=posyError;
            }
        }
        for (int i =0; i < 1; i++) {
            startPosition=currentPosition;
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
            t=0;

            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance=sqrt(pow(point[i+2][1]-currentPosition[1],2)+pow(point[i+2][0]-currentPosition[0],2));
                double oneDistance=sqrt(pow(point[i+1][1]-currentPosition[1],2)+pow(point[i+1][0]-currentPosition[0],2));
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>t){
                    t = (oneDistance+Velocity/6)/(oneDistance+twoDistance);
                }
                if((oneDistance+Velocity/6)/(oneDistance+twoDistance)>1){
                    break;
                }
                target_position[0] = 0.5 * ((2 * point[i+1][0]) + (-point[i+0][0] + point[i+2][0]) * t + (2 * point[i+0][0] - 5 * point[i+1][0] + 4 * point[i+2][0] - point[i+3][0]) * pow(t, 2) +
                        (-point[i+0][0] + 3 * point[i+1][0] - 3 * point[i+2][0] + point[i+3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i+1][1]) + (-point[i+0][1] + point[i+2][1]) * t + (2 * point[i+0][1] - 5 * point[i+1][1] + 4 * point[i+2][1] - point[i+3][1]) * pow(t, 2) +
                        (-point[i+0][1] + 3 * point[i+1][1] - 3 * point[i+2][1] + point[i+3][1]) * pow(t, 3));
                target_position[2] = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                        3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                if((oneDistance+Velocity/4)/(oneDistance+twoDistance)>t){
                    t = (oneDistance)/(oneDistance+twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + ( + point[i + 2][0]-point[i+0][1]) * t + ( - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        ( + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1]-point[i+0][1]) * t + ( - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        ( + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                mpconst=target_position[2];
                target_position[2]=-(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                error = currentPosition[2]-target_position[2];
                error %= 360;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                xError=sqrt(pow((power)*power*30,2)/(1+pow(mpconst,2)))-xVelocity;
                posxError=tarcurpos[0]-currentPosition[0];
                yError=(xError+xVelocity)*mpconst-yVelocity;
                posyError=(tarcurpos[1]-currentPosition[1]);
                yInt += (yError + posyError) * differtime;
                xInt += (xError + posxError) * differtime;
                xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                xCorrection *= -1;
                yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                angleConst = currentPosition[2] * PI / 180;
                double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity = (-error);
                anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 3) / 135;

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
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                motorRightBack.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]),2) + pow(point[i + 2][1] - currentPosition[1],2)));
                pxError=xError;
                pyError=yError;
                pposxError=posxError;
                pposyError=posyError;
            }
        }
        for (int i = 1; i < 2; i++) {
            difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
            t = 0;
            boolean td = true, approximated = false;
            while (op.opModeIsActive() && (abs(difference) >= 0.5)) {
                currentPosition = track();
                double twoDistance = sqrt(pow(point[i + 2][1] - currentPosition[1], 2) + pow(point[i + 2][0] - currentPosition[0], 2));
                double oneDistance = sqrt(pow(point[i + 1][1] - currentPosition[1], 2) + pow(point[i + 1][0] - currentPosition[0], 2));
                if ((oneDistance + Velocity / 6) / (oneDistance + twoDistance) > t && td) {
                    t = (oneDistance + Velocity / 6) / (oneDistance + twoDistance);
                }
                if (t > 1) {
                    t = 1;
                    td = false;
                }
                if (!td) {
                    t = 1;
                    target_position[0] = x3;
                    target_position[1] = y3;
                }
                target_position[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * t + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                target_position[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * t + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                if ((oneDistance + Velocity / 4) / (oneDistance + twoDistance) > t) {
                    t = (oneDistance) / (oneDistance + twoDistance);
                }
                tarcurpos[0] = 0.5 * ((2 * point[i + 1][0]) + (+point[i + 2][0] - point[i + 0][1]) * t + (-5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * pow(t, 2) +
                        (+3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 3));

                tarcurpos[1] = 0.5 * ((2 * point[i + 1][1]) + (point[i + 2][1] - point[i + 0][1]) * t + (-5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * pow(t, 2) +
                        (+3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 3));
                target_position[2] = (0.5 * (+(+point[i + 2][0] - point[i + 0][0]) + 2 * (2 * point[i][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0] - point[i + 3][0]) * t +
                        3 * (-point[i][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0] + point[i + 3][0]) * pow(t, 2)))/(0.5 * (+(+point[i + 2][1] - point[i + 0][1]) + 2 * (2 * point[i][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1] - point[i + 3][1]) * t +
                        3 * (-point[i][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1] + point[i + 3][1]) * pow(t, 2)));
                target_position[2]=-(atan2(target_position[2],1)*180/PI + (direction-1) * 180);
                error = currentPosition[2]-target_position[2];
                error = currentPosition[2] - target_position[2];
                error %= 360;
                approxDifference = 0;
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                xError = sqrt(pow((power) * power *30, 2) / (1 + pow(mpconst, 2))) - xVelocity;
                posxError = tarcurpos[0] - currentPosition[0];
                yError = (xError + xVelocity) * mpconst - yVelocity;
                posyError = (tarcurpos[1] - currentPosition[1]);
                if (difference < 20) {
                    double tdiff = (1 - t) / 20;
                    double[] dummyPosition = {0, 0};
                    double[] dummyPositionTwo = {currentPosition[0], currentPosition[1]};
                    if (!approximated) {
                        for (double j = t; j < 1 + tdiff; j += tdiff) {
                            dummyPosition[0] = 0.5 * ((2 * point[i + 1][0]) + (-point[i + 0][0] + point[i + 2][0]) * j + (2 * point[i + 0][0] - 5 * point[i + 1][0] + 4 * point[i + 2][0]) * pow(j, 2) +
                                    (-point[i + 0][0] + 3 * point[i + 1][0] - 3 * point[i + 2][0]) * pow(j, 3));

                            dummyPosition[1] = 0.5 * ((2 * point[i + 1][1]) + (-point[i + 0][1] + point[i + 2][1]) * j + (2 * point[i + 0][1] - 5 * point[i + 1][1] + 4 * point[i + 2][1]) * pow(j, 2) +
                                    (-point[i + 0][1] + 3 * point[i + 1][1] - 3 * point[i + 2][1]) * pow(j, 3));
                            approxDifference += abs(sqrt(pow((dummyPosition[0] - dummyPositionTwo[0]), 2) + pow(dummyPosition[1] - dummyPositionTwo[1], 2)));
                            dummyPositionTwo = dummyPosition;
                        }
                        //approximated=true;
                    } else {
                        approxDifference -= Velocity * differtime;
                    }
//                    int xCon=0;
//                    if(currentPosition[0]<point[i+2][0]){
//                        xCon=1;
//                    }
//                    else {
//                        xCon = -1;
//                    }
//                    int yCon=0;
//                    if(currentPosition[1]<point[i+2][1]){
//                        yCon=1;
//                    }
//                    else{
//                        yCon=-1;
//                    }
                    //power=startpower;
                    //double trueVelocity = sqrt(xCon*pow(xVelocity,2)+yCon*pow(yVelocity,2));
                    if (approxDifference < 5 * pow(startpower, 2)) {
                        power = max(min(startpower, startpower * approxDifference / 10 / power), 0.4);
                        xError = sqrt((pow(approxDifference, 2) / 1 * power * power) / (1 + pow(mpconst, 2))) - xVelocity;
                        yError = (xError + xVelocity) * mpconst - yVelocity;

                    } else {
                        power = startpower;
                    }
                }
                yInt += (yError + posyError) * differtime;
                xInt += (xError + posxError) * differtime;
                xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError - pposxError - pxError) / differtime;
                xCorrection *= -1;
                yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError - pposyError - pyError) / differtime;
                angleConst = currentPosition[2] * PI / 180;
                double angleInCorrection = atan2(yCorrection, xCorrection) - (angleConst);
                double angleCorrectPower[] = {sin(angleInCorrection + PI / 4), sin(angleInCorrection - PI / 4), sqrt(pow(yCorrection, 2) + pow(xCorrection, 2))};
                angleInRadians = atan2(y, -x) - (angleConst);
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                double targetaVelocity = (-error);
                anglecorrection = power * ((-targetaVelocity + aVelocity) + error * 3) / 135;

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
//                op.telemetry.addData("t", t);
//                op.telemetry.addData("i", i);
                op.telemetry.addData("ytarget", target_position[1]);
                op.telemetry.addData("xtarget", target_position[0]);
//                op.telemetry.addData("angletarget", target_position[2]);
//                op.telemetry.addData("angletarget2", angleInRadians);
//                op.telemetry.addData("mp", mpconst);
//                op.telemetry.addData("error", error);
//                op.telemetry.addData("axisa", axisa);
//                op.telemetry.addData("xerror", xError);
//                op.telemetry.addData("yError", yError);
//                op.telemetry.addData("xvelocity", xVelocity);
//                op.telemetry.addData("yvelocity", yVelocity);
//                op.telemetry.addData("xvelocity",sqrt(pow((power)*power*25,2)/(1+pow(mpconst,2))));
//                op.telemetry.addData("yvelocity", (xError+xVelocity)*mpconst);
//                op.telemetry.addData("xCorrection", xCorrection);
//                op.telemetry.addData("yCorrection",yCorrection);
//                op.telemetry.addData("power",power);
                motorRightBack.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] + anglecorrection));
                motorRightFront.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] + anglecorrection));
                motorLeftBack.setPower((power * anglePower[0] + angleCorrectPower[0] * angleCorrectPower[2] - anglecorrection));
                motorLeftFront.setPower((power * anglePower[1] + angleCorrectPower[1] * angleCorrectPower[2] - anglecorrection));
//            op.telemetry.addData("leftBack",power * anglePower[0] - anglecorrection);
//            op.telemetry.addData("rightBack",power * anglePower[1] + anglecorrection);
//            op.telemetry.addData("leftFront",power * anglePower[1] - anglecorrection);
//            op.telemetry.addData("rightFront",power * anglePower[0] + anglecorrection);

                difference = abs(sqrt(pow((point[i + 2][0] - currentPosition[0]), 2) + pow(point[i + 2][1] - currentPosition[1], 2)));
                pxError = xError;
                pyError = yError;
                pposxError = posxError;
                pposyError = posyError;
            }
        }
        stopAllMotors();


    }


    public void turnInPlace(double target, double power) {
        double currentPosition[]=track();
        double time=0;
        while(abs(currentPosition[2]-target)>2.5) {
            currentPosition=track();
            time+=differtime;
            if(time>2.5){
                break;
            }
            double error=currentPosition[2]-target;
            while(error>180){
                error-=360;
            }
            while(error<180){
                error+=360;
            }
            double targetaVelocity = (error)*2;
            double angleConst=(error*2+(targetaVelocity-aVelocity)/10)/192;
            if(abs(angleConst)*power<0.2){
                angleConst/=(abs(angleConst)*power)/0.2;
            }
            if(abs(aVelocity)>260){
                angleConst=0;
            }
            op.telemetry.addData("angleconst", angleConst);
            op.telemetry.addData("targetaVelocity",targetaVelocity);
            op.telemetry.addData("error",error);
            motorLeftBack.setPower(-power*angleConst);
            motorLeftFront.setPower(-power*angleConst);
            motorRightBack.setPower(power*angleConst);
            motorRightFront.setPower(power*angleConst);
        }
        stopAllMotors();
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
        }
        currentPosition = track();
        turnInPlace(startAngle, 1.0);
        stopAllMotors();
    }

    /*public int getMultiplier(DcMotorEx motor) {
        if (motor == odom1) {
            return odomconst[0];
        } else if (motor == odom2) {
            return odomconst[1];
        } else if (motor == odom3) {
            return odomconst[2];
        } else {
            return 1;
        }
    }*/

    /*public double getRawVelocity(DcMotorEx motor) {
        int multiplier = getMultiplier(motor);
        return motor.getVelocity() * multiplier;
    }*/

   /* private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }*/

    /*public double getCorrectedVelocity(DcMotorEx motor) {
        if (motor == odom1) {
            return inverseOverflow(getRawVelocity(motor), velocity[0]);
        } else if (motor == odom2) {
            return inverseOverflow(getRawVelocity(motor), velocity[1]);
        } else if (motor == odom3) {
            return inverseOverflow(getRawVelocity(motor), velocity[2]);
        } else {
            return getRawVelocity(motor);
        }
    }*/
}

