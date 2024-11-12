package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

/**
 * Manages the DriveTrain of the robot
 * Assumes 4-Wheel Mecanum, has support for both field centric drive and robot centric
 * PreReqs include the CustomPID class, also requires the names on the robot controller to match those in init method
 *
 */
public class DriveTrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double speed = .8;
    private double timer;
    private double xPos;
    private double yPos;
    private double xPosSave;
    private double yPosSave;
    private BNO055IMU imu;
    private DcMotor xOdom;
    private DcMotor yOdom;
    HardwareMap hwMap;

    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;

        this.fl = hwMap.get(DcMotor.class, "fl");
        this.fr = hwMap.get(DcMotor.class, "fr");
        this.bl = hwMap.get(DcMotor.class, "bl");
        this.br = hwMap.get(DcMotor.class, "br");
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.xOdom = hwMap.get(DcMotor.class, "odo1");
        this.yOdom = hwMap.get(DcMotor.class, "odo2");

        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);

        timer = System.nanoTime();
    }
    public void reInitFieldCentric(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);
    }
    public void robotCentricDrive(double leftStickY, double leftStickX, double rightStickX){
        double theta = Math.atan2(-1 * leftStickY, leftStickX);
        double power = Math.hypot(leftStickX, -1 * leftStickY);
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        this.fl.setPower(power * cos/max + rightStickX);
        this.fr.setPower(power * sin/max - rightStickX);
        this.bl.setPower(power * sin/max + rightStickX);
        this.br.setPower(power * cos/max - rightStickX);
    }
    public void fieldCentricDrive(double leftStickY, double leftStickX, double rightStickX, double slow){
        double y = -1 * leftStickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / (denominator * slow);
        double backLeftPower = (rotY - rotX + rightStickX) / (denominator * slow);
        double frontRightPower = (rotY - rotX - rightStickX) / (denominator * slow);
        double backRightPower = (rotY + rotX - rightStickX) / (denominator * slow);

        this.fl.setPower(frontLeftPower*this.speed);
        this.fr.setPower(frontRightPower*this.speed);
        this.bl.setPower(backLeftPower*this.speed);
        this.br.setPower(backRightPower*this.speed);

        // Track Position
        double deltaTime = (System.nanoTime() - timer) / 1e9;

        this.xPos += leftStickX * deltaTime;
        this.yPos += leftStickY * deltaTime;

        timer = System.nanoTime();
    }
    public void driveToLocation(double[] PidConstants, double theta, double distance){
        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theta = Math.toRadians(theta);
        CustomPID distanceControl = new CustomPID(PidConstants);
        double range = 100;
        double currAngle = imu.getAngularOrientation().firstAngle;
        distanceControl.setSetpoint(distance);
        while(!((Math.abs(Math.hypot(xOdom.getCurrentPosition(), yOdom.getCurrentPosition())-distance))<= range/2.0)){
            double[] results = distanceControl.calculateGivenRaw(Math.hypot(xOdom.getCurrentPosition(), yOdom.getCurrentPosition()));
            moveInDirection(theta, results[0]);
            fixAngle(new double[]{1, 200, 0}, currAngle);
        }
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveInDirection(theta, 0);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void rememberAndReturn(boolean remember, double[] PidConstants){
        if (remember){
            this.xPosSave = this.xPos;
            this.yPosSave = this.yPos;
        }else{
            double xDiff = this.xPos - this.xPosSave;
            double yDiff = this.yPos - this.yPosSave;
            double angle = Math.toDegrees(Math.tan(yDiff/xDiff));
            double distance = Math.sqrt(Math.pow(yDiff, 2) + Math.pow(xDiff, 2));
            this.driveToLocation(PidConstants, angle, distance);
        }
    }
    public void fixAngle(double[] PidConstants, double angle) {
        CustomPID angleControl = new CustomPID(PidConstants);
        angleControl.setSetpoint(Math.toRadians(angle));
        double range = Math.PI / 90;
        if(!(Math.abs(imu.getAngularOrientation().firstAngle - Math.toRadians(angle)) <= range / 2.0)) {
            double[] results = angleControl.calculateGivenError(angleWrap(Math.toRadians(angle)-imu.getAngularOrientation().firstAngle));
            this.fl.setPower(-results[0]);
            this.fr.setPower(results[0]);
            this.bl.setPower(-results[0]);
            this.br.setPower(results[0]);
        }
    }
    private  double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public void moveInDirection(double theta, double power){
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);

        this.fl.setPower(power * cos);
        this.fr.setPower(power * sin);
        this.bl.setPower(power * sin);
        this.br.setPower(power * cos);
    }

    public void incSpeed(double inc){
        this.speed = Math.min(Math.max((this.speed+inc), 0.3), 0.8);
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getxOdom() {
        return xOdom;
    }

    public DcMotor getyOdom() {
        return yOdom;
    }

    public double getSpeed(){return this.speed;}
    public double getxPos(){return this.xOdom.getCurrentPosition();}
    public double getyPos(){return this.yOdom.getCurrentPosition();}
}
