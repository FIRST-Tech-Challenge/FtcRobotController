package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class odometry extends LinearOpMode{

    Hware robot;
    DcMotor rightFront, rightBack, leftFront, leftBack;
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        verticalRight = hardwareMap.get(DcMotor.class, "VR");
        verticalLeft =hardwareMap.get(DcMotor.class, "VL");
        horizontal = hardwareMap.get(DcMotor.class, "H");

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        goToPosition(0*COUNTS_PER_INCH,24*COUNTS_PER_INCH,0.5,0,1*COUNTS_PER_INCH);

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }


        public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
            double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            while(opModeIsActive() && distance > allowableDistanceError){
                distance = Math.hypot(distanceToXTarget,distanceToYTarget);
                distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
                distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

                double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
                double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
                double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
                double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();


            }

        }

        /**
         * Calculate the power in the x direction
         * @param desiredAngle angle on the x axis
         * @param speed robot's speed
         * @return the x vector
         */
        private double calculateX(double desiredAngle, double speed) {
            return Math.sin(Math.toRadians(desiredAngle)) * speed;
        }

        /**
         * Calculate the power in the y direction
         * @param desiredAngle angle on the y axis
         * @param speed robot's speed
         * @return the y vector
         */
        private double calculateY(double desiredAngle, double speed) {
            return Math.cos(Math.toRadians(desiredAngle)) * speed;
        }




}
