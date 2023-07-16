package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer.Tracker.aVelocity;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer.Tracker.xVelocity;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer.Tracker.yVelocity;
import static org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.BlackoutRobot.loopTime;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer.Tracker;

//2.0,1.7,1.1
public class MecanumChassis{
    Tracker tracker = null;
    final ElapsedTime runtime = new ElapsedTime();
    private final DcMotorEx motorLeftFront,motorLeftBack,motorRightFront,motorRightBack;
    public MecanumChassis( Tracker trackers) {
        tracker = trackers;

        motorLeftFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");

        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public double getAngle() {
        return tracker.getPos()[2];
    }

    public double[] track() {
        return tracker.getPos();
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
            if(difference<5*startpower){
                powerconst=max(min(startpower, startpower*difference/5),0.3/startpower);
                xError = xVelocity/abs(xVelocity)*((pow(difference,2)/4) / abs(1 - pow(mpconst, 2)) - abs(xVelocity));
                yError = (xError + xVelocity) * mpconst - yVelocity;
                targetspeed = pow(difference,2)/4;
            }
            xCorrection = p * (xError + posxError) + I * xInt + D * (xError + posxError ) / loopTime;
            yCorrection = p * (yError + posyError) + I * yInt + D * (yError + posyError ) / loopTime;
            x = target_position[0] - currentPosition[0];
            y = target_position[1] - currentPosition[1];
            if(x==0){
                x=0.0001;
            }
            if(y==0){
                y=0.0001;
            }
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

        stopAllMotors();
    }
    public void splineToPosition(int direction, double power, double targetAnglu) {
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void splineToPositionHead(int direction, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, boolean start, boolean end, double power) {

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
        double currentPosition[]=track();
        double time=0;
        while(abs(currentPosition[2]-target)>2.5) {
            currentPosition=track();
            time+=loopTime;
            if(time>2.5){
                break;
            }
            double error=currentPosition[2]-target;
            error %= 360;
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }
            double targetaVelocity = (error)*2;
            double angleConst=(error*2+(targetaVelocity+aVelocity)/10)/192;
            if(abs(angleConst)*power<0.23){
                angleConst/=(abs(angleConst)*power)/0.23;
            }
            if(abs(angleConst)*power>0.6){
                angleConst/=(abs(angleConst)*power)/0.6;
            }
            op.telemetry.addData("angleconst", angleConst);
            op.telemetry.addData("targetaVelocity",targetaVelocity);
            motorLeftBack.setPower(-power*angleConst);
            motorLeftFront.setPower(-power*angleConst);
            motorRightBack.setPower(power*angleConst);
            motorRightFront.setPower(power*angleConst);
        }
    }


}
