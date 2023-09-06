package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.PIDNew;
import org.firstinspires.ftc.teamcode.util.Constants;

public class MecanumDriveByGyro extends MecanumDriveImpl implements DriveRobot, Gyro {

    int [] FORWARD_VALUES, REVERSE_VALUES, LATERAL_LEFT_VALUES, LATERAL_RIGHT_VALUES, ROTATE_VALUES;

    double robotHeading;
    double driveSpeed;
    double turnSpeed;
    double headingError;
    Telemetry.Item T_angle;

    private Orientation _lastAngles = new Orientation();
    private double _currentAngle = 0.0;
    protected ImuDevice imu;

    @Override
    public void resetAngle() {
        _lastAngles = imu.getOrientation();
        _currentAngle = 0.0;
    }

    @Override
    public double getAngle() {
        Orientation orientation = imu.getOrientation();
        double deltaAngle = orientation.firstAngle - _lastAngles.firstAngle;
        if (deltaAngle > 180) deltaAngle -= 360;
        if (deltaAngle<=-180) deltaAngle += 360;

        _currentAngle += deltaAngle;
        _lastAngles =  orientation;
        _telemetry.addData("currentGyroZ", orientation.firstAngle);
        return _currentAngle;
    }

    @Override
    public void turn(double degrees) {
        resetAngle();
        double error = degrees;
        while (Math.abs(error) > 2) {
            double motorPower = (error<0)?-0.3:0.3;
            // make it turn!
            error = degrees - getAngle();
            _telemetry.addData("error", error);
        }
        //stop the motors
    }

    @Override
    public void turnTo(double degrees) {
        Orientation orientation = imu.getOrientation();
        double error = degrees - orientation.firstAngle;
        if (error>180) error -=360;
        if (error<=-180) error +=360;
        turn(error);
    }

    @Override
    public double getAbsoluteAngle() {
        return imu.getOrientation().firstAngle;
    }

    public void turnToPID(double degrees) {
        PIDNew mypid;
        mypid = new PIDNew(degrees,1,0,0.003) {
            @Override
            public double getPTerm(double current) {
                double error = degrees - current;
                error = (error%360+360)%360;
                if (error>1) {

                }
                return error;
            }
        };
    }



    public MecanumDriveByGyro(MecanumDriveParameters parameters, ImuDevice imu) {
        super(parameters);
        //init();
        this.imu = imu;
        imu.resetHeading();
        //motor directions RF, RR, LR, LF, rotate
        FORWARD_VALUES = new int[]{ 1, 1, 1, 1,1};
        REVERSE_VALUES = new int[]{-1, -1, -1, -1,-1};
        LATERAL_LEFT_VALUES = new int[]{1,-1,1,-1,1};
        LATERAL_RIGHT_VALUES = new int[]{-1,1,-1,1,1};
        ROTATE_VALUES = new int[]{1,1,-1,-1};

        T_angle = _telemetry.addData("Heading","");
    }


    @Override
    public void outputTelemetry(MecanumDriveTelemetryTypes type) {
        if (type == MecanumDriveTelemetryTypes.HEADING) {
            T_angle.setValue(getAngle());
        }
        else {
            super.outputTelemetry(type);
        }
    }

    @Override
    public void driveForward(double distance) {
        driveRobot(distance, FORWARD_VALUES);
    }

    @Override
    public void driveLeft(double distance) {
        driveRobot(distance, LATERAL_LEFT_VALUES);
    }

    @Override
    public void driveReverse(double distance) {
        driveRobot(distance, REVERSE_VALUES);
    }

    @Override
    public void driveRight(double distance) {
        driveRobot(distance, LATERAL_RIGHT_VALUES);
    }

    public void driveRobot(double inches, int [] direction) {
        // Determine new target position, and pass to motor controller
        int moveCounts = moveCounts(inches);
        setTargetPositions(moveCounts, direction);

        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required driving speed  (must be positive? (nope) for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop

        moveRobotDirection(Constants.DRIVE_SPEED, 0,direction);

        // keep looping while we are still active, and BOTH motors are running.
        while (leftIsBusy() && rightIsBusy()) {

            // Determine required steering to keep on heading
            turnSpeed = direction[4] * getSteeringCorrection(robotHeading, Constants.P_DRIVE_GAIN);

            // Apply the turning correction to the current driving speed.
            moveRobotDirection(driveSpeed, turnSpeed, direction);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobotDirection(0, 0, FORWARD_VALUES);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        imu.getHeading();

        /*
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        //robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - imu.getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        */

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return 0;//Range.clip(headingError * proportionalGain, -1, 1);
    }

    @Override
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        // keep looping while we have time remaining.
        while (holdTimer.time() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobotDirection(0, turnSpeed,FORWARD_VALUES);
        }
        // Stop all motion;
        moveRobotDirection(0, 0,FORWARD_VALUES);
    }

    public boolean leftIsBusy() {  // return true if either is busy.
        return (_motors[1].isBusy() || _motors[3].isBusy());
    }

    public int moveCounts(double distance) {
        return (int)(distance * Constants.COUNTS_PER_INCH_FORWARD);
    }

    public void moveRobotDirection(double power, double rotate, int [] direction) {
        driveSpeed = power;
        turnSpeed = rotate;
        rotate *= direction[4];
        double [] wheelSpeeds = {
                direction[0]*power - rotate,   // Front Right
                direction[1]*power - rotate, // Rear Right
                direction[2]*power + rotate,   // Rear Left
                direction[3]*power + rotate  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        outputTelemetry(MecanumDriveTelemetryTypes.WHEEL_POSITIONS);
        outputTelemetry(MecanumDriveTelemetryTypes.WHEEL_SPEEDS);
        outputTelemetry(MecanumDriveTelemetryTypes.HEADING);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        imu.resetHeading();
        robotHeading = 0;
    }

    public boolean rightIsBusy() {
        return (_motors[0].isBusy() || _motors[2].isBusy());
    }

    public int rotateCounts(double degrees) {
        return (int)Math.round((degrees * Constants.COUNTS_PER_ROTATE)/360);
    }

    public void setTargetPositions(int target, int [] directions) {
        for (int i=0; i<_ENCODER_WHEELS.length; i++) {
            _motors[i].setTargetPosition(_motors[i].getCurrentPosition() + target * directions[i]);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        int loop_count=0;

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, Constants.P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while ( (loop_count++ < 10) && (Math.abs(headingError) > Constants.HEADING_THRESHOLD) ) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobotDirection(0, turnSpeed, FORWARD_VALUES);
        }

        // Stop all motion;
        moveRobotDirection(0, 0, FORWARD_VALUES);
    }

    // CCW == positive degrees
    // CW == negative degrees
    public void turnRobot(double degrees) {
// Determine new target position, and pass to motor controller

        int moveCounts = rotateCounts(degrees);

        setTargetPositions(moveCounts, ROTATE_VALUES);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required driving speed  (must be positive? (nope) for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        moveRobotDirection(Constants.DRIVE_SPEED, 0,ROTATE_VALUES);
        // keep looping while we are still active, and motors are running.
        do {} while (leftIsBusy() && rightIsBusy());
        moveRobotDirection(0, 0, FORWARD_VALUES);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
