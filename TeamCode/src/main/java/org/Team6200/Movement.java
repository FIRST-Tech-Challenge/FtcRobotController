package org.Team6200;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashMap;

public class Movement {
    private HashMap<String, HardwareDevice> motors;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    public boolean isDoingAction = false;
    private Thread currentThread = null;
    public Movement(HashMap<String, HardwareDevice> motorsC){
        this.motors = motorsC;
        frontLeft = (DcMotor)motors.get("frontLeft");
        frontRight = (DcMotor)motors.get("frontRight");
        backRight = (DcMotor)motors.get("backRight");
        backLeft = (DcMotor)motors.get("backLeft");
        imu = (BNO055IMU)motors.get("imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }

    public void turnToDeg(int degrees){
        int currentDeg = MathUtil.orientationToDegrees(imu.getAngularOrientation().firstAngle);
        if(currentDeg != degrees){
            isDoingAction = true;
            if(currentDeg < degrees){
                sharpLeft();
            }else{
                sharpRight();
            }
            if(currentThread!=null){
                currentThread.stop();
            }

            currentThread = new Thread(()->{
                int nCurrentDeg = MathUtil.orientationToDegrees(imu.getAngularOrientation().firstAngle);
                int offset = 2;
                if(currentDeg < degrees){
                    offset = -2;
                }
                while(nCurrentDeg != degrees + offset){
                    nCurrentDeg = MathUtil.orientationToDegrees(imu.getAngularOrientation().firstAngle);
                }
                isDoingAction = false;
                stopMotors();
            });
            currentThread.start();
        }
    }

    public void turnDegrees(int degrees){
        int currentDeg = MathUtil.orientationToDegrees(imu.getAngularOrientation().firstAngle);
        turnToDeg(currentDeg + degrees);
    }

    public void processJoystick(double x, double y, double rx){
        if(!isDoingAction){
            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx), 1);
            double fr = (y-x-rx)/denominator;
            double fl = (y+x+rx)/denominator;
            double br = (y+x-rx)/denominator;
            double bl = (y-x+rx)/denominator;
            //System.out.println(fr+":"+fl+":"+br+":"+bl);
            setPower(fr, fl, br, bl);
        }
    }

    public void startForward(){
        frontRight.setPower(1);
        frontLeft.setPower(-1);
        backRight.setPower(1);
        backLeft.setPower(-1);
    }

    public void setPower(double fr, double fl, double br, double bl){
        frontRight.setPower(1*fr);
        frontLeft.setPower(-1*fl);
        backRight.setPower(1*br);
        backLeft.setPower(-1*bl);
    }
/*
    public void startForwardPower(double power){
        motors.get("frontRight").setPower(1*power);
        motors.get("frontLeft").setPower(-1*power);
        motors.get("backRight").setPower(1*power);
        motors.get("backLeft").setPower(-1*power);
    }
*/
    public void stopMotors(){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void forwardForMillis(long millis){
        startForward();
        new Thread(()->{
            try {
                Thread.sleep(millis);
                this.stopMotors();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }
/*
    public void sharpTurnPower(double power){
        motors.get("frontRight").setPower(power);
        motors.get("frontLeft").setPower(power);
        motors.get("backRight").setPower(power);
        motors.get("backLeft").setPower(power);
    }
*/
    public void sharpLeft(){

        frontRight.setPower(1);
        frontLeft.setPower(1);
        backRight.setPower(1);
        backLeft.setPower(1);
    }

    public void sharpRight(){
        frontRight.setPower(-1);
        frontLeft.setPower(-1);
        backRight.setPower(-1);
        backLeft.setPower(-1);
    }
/*
    public void sRightForMillis(long millis){
        sharpRight();
        new Thread(()->{
            try {
                Thread.sleep(millis);
                this.stopMotors();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }

    public void sLeftForMillis(long millis){
        sharpLeft();
        new Thread(()->{
            try {
                Thread.sleep(millis);
                this.stopMotors();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }).start();
    }
*/

}
