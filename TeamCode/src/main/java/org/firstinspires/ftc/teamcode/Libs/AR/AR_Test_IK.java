package org.firstinspires.ftc.teamcode.Libs.AR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "AR_Test_IK", group = "Linear Opmode")

public class AR_Test_IK extends LinearOpMode {



    // Hardware variables

    private DcMotor shoulderMotor;

    private DcMotor elbowMotor;



    // Arm dimensions (in some consistent units, e.g., cm)

    private final double L1 = 22.0; // Length of first arm segment

    private final double L2 = 16.0; // Length of second arm segment



    @Override

    public void runOpMode() {

        // Initialize motors

        shoulderMotor = hardwareMap.get(DcMotor.class, "shoulderMotor");

        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");

        // Wait for the start button

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Example target coordinates (x, y) for the arm
            double targetX = 40.0;
            double targetY = 10.0;

            int startJoint1 = 90;
            int startJoint2= 90;

            // Set up the arm to the start position from its folded form
            moveMotorToAngle(shoulderMotor, startJoint1);
            moveMotorToAngle(elbowMotor, startJoint2);


            // Calculate joint angles using inverse kinematics
            double[] angles = calculateJointAngles(targetX, targetY);

            if (angles != null) {
                double shoulderAngle = angles[0]; // in degrees
                double elbowAngle = angles[1];   // in degrees

                // Move the motors to the calculated angle
                moveMotorToAngle(shoulderMotor, shoulderAngle);
                moveMotorToAngle(elbowMotor, elbowAngle);

                // Display telemetry

                telemetry.addData("Target X", targetX);

                telemetry.addData("Target Y", targetY);

                telemetry.addData("Shoulder Angle", shoulderAngle);

                telemetry.addData("Elbow Angle", elbowAngle);

                telemetry.update();

            } else {

                telemetry.addData("Error", "Target unreachable");

                telemetry.update();

            }
            sleep(2000); // Pause to observe motion
        }
    }



    /**

     * Calculate the joint angles using inverse kinematics.

     *

     * @param x Target x-coordinate of the end effector.

     * @param y Target y-coordinate of the end effector.

     * @return Array of joint angles [shoulderAngle, elbowAngle] in degrees, or null if unreachable.

     */

    private double[] calculateJointAngles(double x, double y) {

        double distance = Math.sqrt(x * x + y * y);
        // Check if the target is reachable
        if (distance > (L1 + L2) || distance < Math.abs(L1 - L2)){
            return null; // Target is unreachable
        }



        // Law of cosines for elbow angle
        double cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        if (cosTheta2 > -1 || cosTheta2 < 1) {
            return null;
        }

        try {
            double theta2 = Math.acos(cosTheta2); // Elbow angle in radians
        } catch(Exception e){
            double theta2 =
        }



        // Law of cosines and trigonometry for shoulder angle

        double theta1 = Math.atan2(y, x) - Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));



        // Convert radians to degrees

        double shoulderAngle = Math.toDegrees(theta1);

        double elbowAngle = Math.toDegrees(theta2);



        return new double[] { shoulderAngle, elbowAngle };

    }



    /**

     * Moves a motor to a specified angle.

     *

     * @param motor The motor to move.

     * @param angle The target angle in degrees.

     */

    private void moveMotorToAngle(DcMotor motor, double angle) {

        // Convert the angle to encoder counts (adjust based on motor setup)

        int targetPosition = (int) (angle * (1440 / 360.0)); // Example: 1440 counts per revolution

        motor.setTargetPosition(targetPosition);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0.5); // Set motor power

    }

}