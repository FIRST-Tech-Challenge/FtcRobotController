package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.purepursuit.LineSegment;
import org.firstinspires.ftc.teamcode.purepursuit.PurePursuit;

public class MecanumPurePursuitController {
    private PurePursuit purePursuit;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime timer;

    public double current_X = 0;

    public double current_Y = 0;
    private static final double WHEEL_RADIUS = 2.0; // inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REVOLUTION = 537.7; // for GoBILDA 5202 Series
    private static final double MAX_VELOCITY = 86.6; //inches/sec

    public MecanumPurePursuitController(HardwareMap hardwareMap, double startX, double startY) {
        purePursuit = new PurePursuit(startX, startY);

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "LF");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backLeft = hardwareMap.get(DcMotor.class, "LB");
        backRight = hardwareMap.get(DcMotor.class, "RB");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        timer = new ElapsedTime();
    }

    public void followPath() {

        double targetAngle = purePursuit.calculateMovementAngle();
        double lastTime = timer.seconds();
        targetA = targetAngle;

        // Decompose movement into drive and strafe components
        double drive = MAX_VELOCITY * Math.sin(targetAngle);  // Forward/backward component
        double strafe = MAX_VELOCITY * Math.cos(targetAngle); // Left/right component

        // Calculate individual motor powers using the correct mecanum formulas
        double frontLeftPower = drive - strafe; //1 "+" and "-" were reversed to correct strafe directions
        double frontRightPower = drive + strafe; //-1
        double backLeftPower = drive + strafe; //-1
        double backRightPower = drive - strafe; //1

        // Normalize powers
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Update robot position
        double currentTime = timer.seconds();
        double timeSlice = currentTime - lastTime;
        purePursuit.updatePositions(MAX_VELOCITY, targetAngle, timeSlice);
        //lastTime = currentTime;

        current_X = purePursuit.robotX;
        current_Y = purePursuit.robotY;
        fl = frontLeftPower;
        fr = frontRightPower;
        br = backRightPower;
        bl = backLeftPower;
        dxReceiver = purePursuit.DX;
        return;
    }
    double targetA;
    double dxReceiver;

    // Additional helper method for complex movements
    public void setMecanumDrive(double drive, double strafe, double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize powers
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public boolean isPathComplete() {
        double distanceToEnd = Math.sqrt(
                Math.pow(purePursuit.robotX - lastPathEndX, 2) +
                        Math.pow(purePursuit.robotY - lastPathEndY, 2)
        );
        return distanceToEnd < 0.1;
    }

    double fl;
    double fr;
    double br;
    double bl;

    private double lastPathEndX, lastPathEndY;

    public void setPathEndPoint(double x, double y) {
        lastPathEndX = x;
        lastPathEndY = y;
    }

    public void addPathSegment(double x0, double y0, double x1, double y1) {
        purePursuit.addSegmentToPath(new LineSegment(x0, y0, x1, y1));
    }
}