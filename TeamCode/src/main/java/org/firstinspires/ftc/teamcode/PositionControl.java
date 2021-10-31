package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//@Disabled
public class PositionControl {
    MecanumChassis robot;
    private ElapsedTime runtime = new ElapsedTime();

    private volatile double[] robotPos = new double[3];
    private volatile double[] robotTargetPos = new double[3];
    private double targetVd = 0;
    private double targetVw = 0;
    private double lastVd = 0;
    private double lastVw = 0;
    private double robot_vd = 0;
    private double robot_vw = 0;





    // need test and calibrate, control period = 20ms,
    private final double MAX_ACCEL = 0.06;   //??25 * MAX_ACCEL = 1.568, 0.5 second to accelerate to max speed from "0"
    private final double MAX_ANGLEACCEL = 0.2;  //
    private final double MAX_VD = 2.59;        //2.59m/s for 1150 RPM(ratio 2:1) 1.8 for 435rpm only for theory, on floor, it only around 85%, so not set vel over 1.8m/s.  0.096 diameter of wheel 2.18 m/s = 435 /60 * PI * 0.096  1.568 m/s = 312/60 * PI * 0.096
    private final double MAX_VW = 2 * Math.PI;          // 1 round per second 360 degree per second
    private final double MIN_VD = 0.1;          // 0.2m/s if kpd = 2, it means in 10 cmd it will drive in min speed
    private final double MIN_VW = 15 * Math.PI /180;          // 10 degree per second
    private final double DIS_DEADBAND = 0.02;                   // 2 cm
    private final double ANGLE_DEADBAND = 2.0 / 180 * Math.PI;  // 1 degree error
    //private double kpd = 2.0;
    private double kpw = 1.0;       // here means around 9.7 degree robot will enter min_vw speed to turn
    //we will use the acceleration curve to control the speed
    private final double BRAKE_DIS = 1.5;   //1.5M brake from Vmax to Vmin
    private final double BRAKE_RATIO = (MAX_VD - MIN_VD) / BRAKE_DIS;
    private double brake_dis = 0;
    private final double BRAKE_ANGLE = 180.0 * Math.PI / 180;   //90 degree brake from MAX_VW to _MIN_VW
    private final double BRAKE_ANGLE_RATIO = (MAX_VW - MIN_VW) / BRAKE_ANGLE;
    private double brake_angle = 0;
    private double disDeadZone = DIS_DEADBAND;
    private double angleDeadZone = ANGLE_DEADBAND;

    //same define in position estimation.java, may put somewhere together
    private static final double DIS_2WHEELS = 0.415;          // for 4 mecanum wheels, actually test value is 0.415 distance between 2 wheels, for encoder turning
    private static final double DIS_L1 = DIS_2WHEELS /2;      // left-right wheels to center distance
    private static final double DIS_L2 = 0.168;                //for gobila, 7*24mm, 7 big hole distance,front-rear wheels to center distance

    private static final double WHEEL_ANGLE = 42.5/ 180 * Math.PI;        //Math.PI /4;     //mecanum wheel roller angle, may need calibrate

    private final long THREAD_PERIOD = 10; // 10ms control period
    private boolean taskDone = false;

    private PositionControl.PositionControlThread positionControlThread = null;

    private PositionEstimation positionEstimation = null;

    PositionControl(MecanumChassis robot,  PositionEstimation positionEstimation){
        this.robot = robot;
        this.runtime = new ElapsedTime();
        this.positionEstimation = positionEstimation;
        // set current angle first

    }

    public void stopRobot()
    {
        this.robot.rightFrontDrive.setPower(0);
        this.robot.leftFrontDrive.setPower(0);
        this.robot.rightRearDrive.setPower(0);
        this.robot.leftRearDrive.setPower(0);
        this.taskDone = true;
    }

    /* set the robot command out*/
    public void driveRobot(double leftPowerF, double rightPowerF, double leftPowerR, double rightPowerR){
        //here if we use setPower function, we need normalize the speed
        //or may use the setVelocity() function, need transform to radian
        /*
        leftPowerF = (Math.abs(leftPowerF) < 0.1? Math.signum(leftPowerF) * 0.1: leftPowerF);
        leftPowerR = (Math.abs(leftPowerR) < 0.1? Math.signum(leftPowerR) * 0.1: leftPowerR);
        rightPowerF = (Math.abs(rightPowerF) < 0.1? Math.signum(rightPowerF) * 0.1: rightPowerF);
        rightPowerR = (Math.abs(rightPowerR) < 0.1? Math.signum(rightPowerR) * 0.1: rightPowerR);
*/
        leftPowerF    = Range.clip(leftPowerF/MAX_VD , -1.0, 1.0) ;
        rightPowerF   = Range.clip(rightPowerF/MAX_VD, -1.0, 1.0) ;
        leftPowerR    = Range.clip(leftPowerR/MAX_VD, -1.0, 1.0) ;
        rightPowerR   = Range.clip(rightPowerR /MAX_VD, -1.0, 1.0) ;
        this.robot.rightFrontDrive.setPower(rightPowerF);
        this.robot.rightRearDrive.setPower(rightPowerR);
        this.robot.leftRearDrive.setPower(leftPowerR);
        this.robot.leftFrontDrive.setPower(leftPowerF);
    }
    public boolean checkTaskDone(){
        return this.taskDone;
    }

    public double debug_task(){
        return  robot_vd;
    }

    public void goToTargetPosition(double[] setTarget, double vd, double vw, double disRes, double angleRes){
        this.positionControlThread = null;
        this.robotTargetPos = setTarget;
        this.targetVd = Math.abs(vd);
        if (this.targetVd < MIN_VD) this.targetVd = MIN_VD;
        if (this.targetVd > MAX_VD) this.targetVd = MAX_VD;
        brake_dis = (this.targetVd - MIN_VD) / BRAKE_RATIO;

        this.disDeadZone = Math.abs(disRes );
        this.angleDeadZone = Math.abs(angleRes* Math.PI / 180 );   //2*


        this.targetVw = Math.abs(vw);
        if (this.targetVw < MIN_VW) this.targetVw = MIN_VW;
        if (this.targetVw > MAX_VW) this.targetVw = MAX_VW;
        brake_angle = Math.abs(this.targetVw - MIN_VW) / BRAKE_ANGLE_RATIO;

        this.lastVw = 0;
        positionControlThread = new PositionControlThread();
        positionControlThread.start();
    }


    private class PositionControlThread extends Thread
    {

        public PositionControlThread()
        {
            taskDone = false;

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            double disError = 0;
            double angleError = 0;
            double phi = 0;
            double vX = 0;
            double vY = 0;
            double v1 = 0, v2 = 0, v3 = 0, v4 = 0;

            try
            {
                while (!isInterrupted() && (!taskDone))
                {
                    robotPos = positionEstimation.getRobotPos();

                    disError = Math.sqrt(Math.pow((robotTargetPos[0] - robotPos[0]), 2) + Math.pow((robotTargetPos[1] - robotPos[1]), 2));
                    angleError = robotTargetPos[2] - robotPos[2];

                    if ((disError <= disDeadZone) && (Math.abs(angleError) <= angleDeadZone)){
                        //stop robot
                        taskDone = true;
                        stopRobot();
                    }
                    else{
                        if ((disError) > disDeadZone){
                            if((disError) < brake_dis){
                                // here will follow the brake curve
                                robot_vd = Math.abs(disError) * BRAKE_RATIO + MIN_VD;
                            }
                            else{
                                // will follow the accel curve
                                robot_vd = lastVd + MAX_ACCEL;
                            }
                            robot_vd = (robot_vd > targetVd? targetVd: robot_vd);
                            robot_vd = (robot_vd < MIN_VD? MIN_VD: robot_vd);
                        }
                        else{
                            robot_vd = 0;
                        }

                        lastVd =  robot_vd;


                        if (Math.abs(angleError) > angleDeadZone){
                            if(Math.abs(angleError) < brake_angle){
                                //normal should run this
                                robot_vw = Math.signum(angleError) * (Math.abs(angleError) * BRAKE_ANGLE_RATIO + MIN_VW);
                            }
                            else{
                                robot_vw =  Math.signum(angleError) * (Math.abs(lastVw) + MAX_ANGLEACCEL);
                            }

                            if (Math.abs(robot_vw) > Math.abs(targetVw))  robot_vw = Math.signum(robot_vw) * Math.abs(targetVw);
                            if (Math.abs(robot_vw) < MIN_VW)  robot_vw = Math.signum(robot_vw) * MIN_VW;
                        }
                        else{
                            robot_vw = 0;
                        }
                        lastVw = robot_vw;



                        phi = Math.atan2((robotTargetPos[1] - robotPos[1]), (robotTargetPos[0]- robotPos[0]));
                        vX = robot_vd * Math.cos(phi - robotPos[2]);
                        vY = robot_vd * Math.sin(phi - robotPos[2]);

                        v1 = vX - vY * (1 / Math.tan(WHEEL_ANGLE)) - (DIS_L1 * Math.tan(WHEEL_ANGLE) + DIS_L2) / Math.tan(WHEEL_ANGLE) * robot_vw;
                        v2 = vX + vY * (1 / Math.tan(WHEEL_ANGLE)) + (DIS_L1 * Math.tan(WHEEL_ANGLE) + DIS_L2) / Math.tan(WHEEL_ANGLE) * robot_vw;
                        v3 = vX + vY * (1 / Math.tan(WHEEL_ANGLE)) - (DIS_L1 * Math.tan(WHEEL_ANGLE) + DIS_L2) / Math.tan(WHEEL_ANGLE) * robot_vw;
                        v4 = vX - vY * (1 / Math.tan(WHEEL_ANGLE)) + (DIS_L1 * Math.tan(WHEEL_ANGLE) + DIS_L2) / Math.tan(WHEEL_ANGLE) * robot_vw;

                        driveRobot(v1, v2, v3,v4);
                    }





                    Thread.sleep(THREAD_PERIOD);
                    //Thread.yield();
                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {}

        }
    }
}