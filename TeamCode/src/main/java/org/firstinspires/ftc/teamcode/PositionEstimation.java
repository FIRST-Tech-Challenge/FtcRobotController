package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.opencv.core.Mat;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class PositionEstimation {
    private static final double     COUNTS_PER_MOTOR_REV    = 145.6    ;// 435RPM-383.6, 1150RPM--145.6
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // the new one is 1:1, mecanum drive chassis
    private static final double     WHEEL_DIAMETER_M   = 0.1;//0.096;     // From gobilda mecanum drawingm should be 0.096
    private static final double     COUNTS_PER_M         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_M * Math.PI);
    //same define in positioncontrol.java, may move to some where
    private static final double DIS_2WHEELS = 0.415;          // for 4 mecanum wheels, actually test value is 0.415 distance between 2 wheels, for encoder turning
    private static final double DIS_L1 = DIS_2WHEELS /2;      // left-right wheels to center distance
    private static final double DIS_L2 = 0.168;                //for gobila, 7*24mm, 7 big hole distance,front-rear wheels to center distance

    private static final double WHEEL_ANGLE = 42.5/ 180 * Math.PI;        //Math.PI /4;     //mecanum wheel roller angle, may need calibrate

    // for thread control
    private volatile Orientation currentAngles;        //used for control
    private volatile double yawReading ;
    private volatile double lastYawReading;
    private volatile double headingAngle ;

    private double lfStartPos = 0, rfStartPos = 0, lrStartPos = 0, rrStartPos = 0;
    private MecanumChassis robot;
    private ElapsedTime runtime = new ElapsedTime();


    private volatile double[] robotPos = new double[3];         //x,y orientation
    private volatile double[] robotPosLast = new double[3];    //last time robot position

    private PositionEstimation.EstimationThread estimationThread = null;

    private static final long THREAD_PERIOD = 10 ;  // 10 ms period to estimate the robot position
    private double wheelEstimateAngle = 0;


    public PositionEstimation(MecanumChassis robot){
        this.robot = robot;
        this.runtime = new ElapsedTime();


        // set current angle first
        currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawReading =  AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);
        lastYawReading = yawReading;
        headingAngle = yawReading;
        wheelEstimateAngle = yawReading / 180 * Math.PI;
        estimationThread = new PositionEstimation.EstimationThread();

        //start the robot position
        estimationThread.start();

    }

    public double[] getRobotPos(){
        return this.robotPos;
    }
    //this function is for compare wheel estimate angle with IMU estimate angle
    public double getWheelEstimateAngle(){
        return  this.wheelEstimateAngle;
    }

    public double getHeadingAngle(){
        double heading = 0;
        currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawReading = AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);

        double delta = yawReading - lastYawReading;
        if (abs(delta) >= 180){
            //it should be over +180 to -180 point
            if (lastYawReading < 0) {
                delta = yawReading - 180 - (180 + lastYawReading);
            }
            if (lastYawReading > 0){
                delta = 180 - lastYawReading + 180 + yawReading;
            }

        }
        headingAngle = headingAngle + delta;
        lastYawReading = yawReading;
        return headingAngle * Math.PI / 180.0;
    }


    private class EstimationThread extends Thread
    {
        private double distance = 0;
        private double setAngle = 0;
        private double setPower = 0;

        private double lfStartPos = 0, rfStartPos = 0, lrStartPos = 0, rrStartPos = 0;
        private double accelerateDis = 0;  // accelerate distance
        public EstimationThread()
        {

            this.lfStartPos = robot.leftFrontDrive.getCurrentPosition();
            this.rfStartPos = robot.rightFrontDrive.getCurrentPosition();
            this.lrStartPos = robot.leftRearDrive.getCurrentPosition();
            this.rrStartPos = robot.rightRearDrive.getCurrentPosition();
            robotPos[0] = 0;    //x    //start as (0,0,heading) robot position
            robotPos[1] = 0;    //y
            robotPos[2] = getHeadingAngle();        //orientation
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            double d1 = 0, d2 = 0, d3 = 0, d4 = 0;
            double dx = 0, dy = 0, dw = 0;
            double wheel_beta = Math.tan(WHEEL_ANGLE) / ( 4 * (DIS_L1 * Math.tan(WHEEL_ANGLE) + DIS_L2));
            try
            {
                while (!isInterrupted())
                {
                    //here we keep driving the robot to set distance with set angle
                    d1 = robot.leftFrontDrive.getCurrentPosition() - this.lfStartPos;
                    d1 = d1 / COUNTS_PER_M;
                    this.lfStartPos = robot.leftFrontDrive.getCurrentPosition();

                    d2 = robot.rightFrontDrive.getCurrentPosition() - this.rfStartPos;
                    d2 = d2 / COUNTS_PER_M;
                    this.rfStartPos = robot.rightFrontDrive.getCurrentPosition();

                    d3 = robot.leftRearDrive.getCurrentPosition() - this.lrStartPos;
                    d3 = d3 / COUNTS_PER_M;
                    this.lrStartPos = robot.leftRearDrive.getCurrentPosition();

                    d4 = robot.rightRearDrive.getCurrentPosition() - this.rrStartPos;
                    d4 = d4 / COUNTS_PER_M;
                    this.rrStartPos = robot.rightRearDrive.getCurrentPosition();

                    dx = (d1 + d2 + d3 + d4) / 4;
                    dy = (-d1 + d2 + d3 - d4) / 4 * Math.tan(WHEEL_ANGLE);
                    dw = (-d1 + d2 - d3 + d4) * wheel_beta ;
                    //for robot heading, we could use the IMU sensor instead of encoder estimation
                    wheelEstimateAngle = wheelEstimateAngle + dw;//this one is for compare
                    //robotPos[2] = robotPos[2] + dw;  //not using encoder
                    robotPos[2] = getHeadingAngle();  //here we force to use the IMU sensor,
                    //translate to globle coordinate system
                    robotPos[0] = robotPos[0] + dx * Math.cos(robotPos[2]) - dy * Math.sin(robotPos[2]);
                    robotPos[1] = robotPos[1] + dx * Math.sin(robotPos[2]) + dy * Math.cos(robotPos[2]);
                    Thread.sleep(THREAD_PERIOD);
                    //Thread.yield();
                }

            }

            catch (Exception e) {}

        }
    }

}