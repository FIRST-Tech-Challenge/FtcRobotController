package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Autonomous extends LinearOpMode {
    MecanumChassis robot = new MecanumChassis();

    private ElapsedTime runtime = new ElapsedTime();
    private PositionEstimation positionEstimation = null;
    private PositionControl positionControl = null;
    private volatile double[] robotPos = new double[3];
    private volatile double[] targetPos = new double[3];

    int[] armPos = {1,2,3};

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        this.positionEstimation = new PositionEstimation(robot);
        this.positionControl = new PositionControl(robot, this.positionEstimation);
        this.positionControl.stopRobot();
        telemetry.addData("Status", "Initialized");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while (!isStopRequested() && !robot.imu.isGyroCalibrated() &&
                !robot.imu.isAccelerometerCalibrated() &&
                !robot.imu.isMagnetometerCalibrated() &&
                !robot.imu.isSystemCalibrated()) {
            idle();
        }
        telemetry.addData("IMU Mode", "IMU calibrating done");
        telemetry.update();

        waitForStart();

        int DuckPos;
        // TensorFlow find duck

        goToWayPoint(0,1,0,0.7,0,0.01,2);

    }
    private void goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes) throws InterruptedException {
        targetPos[0] = x;//1.5;  //x
        targetPos[1] = y;//-0.6;   //y
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes);
        while(!this.positionControl.checkTaskDone()){
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0], robotPos[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
            }
            telemetry.update();
        }
        Thread.sleep(200);

    }
}
