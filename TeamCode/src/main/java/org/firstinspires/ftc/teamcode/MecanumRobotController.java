// This is the robot controller class. This class will contain all methods needed to control the robot
// so you can just instantiate this and use it in other files. Some examples include driving,
// getting the current heading, and moving any other motors on the robot.

// Remember to comment with BERP:
//      B: Behavior, what the method does.
//      E: Exceptions, what exceptions the method throws.
//      R: Return, what the method returns.
//      P: Parameters, the list of parameters and explanations of them.

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumRobotController {
    // Need to find this.
    public static final double COUNTS_PER_INCH = 30.59;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private IMU gyro;

    // Create the controller with all the motors needed to control the robot. If another motor,
    // servo, or sensor is added, put that in here so the class can access it.
    public MecanumRobotController(DcMotor backLeft, DcMotor backRight,
                                  DcMotor frontLeft, DcMotor frontRight,
                                  IMU gyro) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.gyro = gyro;
    }

    // TODO: Implement turn correction on the move function.
    // Behavior: Moves the robot using a given forward, strafe, and turn power.
    // Params:
    //      - double forward: The forward power for the robot.
    //      - double strafe: The strafe power for the robot.
    //      - double turn: The turn power fo the robot.
    public void move(double forward, double strafe, double turn) {
        double backLeftPower = Range.clip((forward + strafe - turn) / 3, -1.0, 1.0);
        double backRightPower = Range.clip((forward + strafe + turn) / 3, -1.0, 1.0);
        double frontLeftPower = Range.clip((forward - strafe - turn) / 3, -1.0, 1.0);
        double frontRightPower = Range.clip((forward - strafe + turn) / 3, -1.0, 1.0);

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

    // TODO: Find exact values for distance and time and implement them to make this method precise.
    // Behavior: Drives the robot a given distance in a given direction without turning it.
    // Params:
    //      - double distance: The distance to drive the robot in inches.
    //      - double direction: The direction, in degrees, that the robot will drive in. This is
    //                          based on the direction the robot was initialized in.
    //      - double time: The amount of time in milliseconds it should take to drive the wanted
    //                     distance. Basically the speed but more precise.
    public void drive(double distance, double direction, double time) {
        double currentHeading = getAngleImuDegrees();

        // This still needs testing.
        double forward = ((Math.cos(direction) * Math.cos(currentHeading * (Math.PI / 180))) +
                (Math.sin(direction) * Math.sin(currentHeading * (Math.PI / 180))));
        double strafe = ((Math.cos(direction) * Math.sin(currentHeading * (Math.PI / 180))) -
                (Math.sin(direction) * Math.cos(currentHeading * (Math.PI / 180))));

        int forwardCounts = (int)(forward * distance * COUNTS_PER_INCH);
        int strafeCounts = (int)(strafe * distance * COUNTS_PER_INCH);

        int backLeftTarget = backLeft.getCurrentPosition() + forwardCounts + strafeCounts;
        int backRightTarget = backRight.getCurrentPosition() + forwardCounts + strafeCounts;
        int frontLeftTarget = frontLeft.getCurrentPosition() + forwardCounts - strafeCounts;
        int frontRightTarget = frontRight.getCurrentPosition() + forwardCounts - strafeCounts;

        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Needs troubleshooting probably. Don't know if I can call isOpModeActive() either so idk
        // how that works.
        while (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
            // Regulates speed, need to make this consistent.
            move(1.0, 0.0, 0.0);
        }

        move(0, 0, 0);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // TODO:
    // Behavior: Turns the robot to a given angle
    //      - double degrees: The angle to turn the robot to in degrees.
    //      - double speed: The speed at which the robot should turn.
    public void turnTo(double angle, double time) {

    }

    // Behavior: Normalizes a given degree value to the range (-180, 180]
    // Returns: The normalized degrees as a double.
    // Params:
    //      - double degrees: The degrees to be normalized.
    public static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    // Behavior: Get the current IMU heading in degrees.
    // Returns: The current robots angle in degrees on the range (-180, 180]
    public double getAngleImuDegrees() {
        return normalize(
                gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
