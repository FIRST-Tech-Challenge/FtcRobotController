package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.GoBildaPinpointDriver;

public class Drive_base {

    AutoConstants ac = new AutoConstants();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;

    public GoBildaPinpointDriver odometry = null;
    double xoffset = -25.4;
    double yoffset = -203.2;
    Pose2D targetPosition = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    double kPX = .01;
    double kPY = .01;
    double kPHeading = .01;
    double errorX;
    double errorY;
    double errorHeading;
    double XYThreshold = .25;
    double headingThreshold = 5;

    void init(HardwareMap hwMap) {
        frontleftDrive  = hwMap.get(DcMotor.class, "fld");
        frontrightDrive = hwMap.get(DcMotor.class, "frd");
        backleftDrive  = hwMap.get(DcMotor.class, "bld");
        backrightDrive = hwMap.get(DcMotor.class, "brd");

     backleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        odometry = hwMap.get(GoBildaPinpointDriver.class,"ODO");
        odometry.setOffsets(xoffset,yoffset);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();
        odometry.setPosition(ac.startingPose);
        setTargetPosition(ac.startingPose);
    }

    void drive(double y, double x, double rx) {
        double frontleftPower;
        double frontrightPower;
        double backleftPower;
        double backrightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontleftPower = (y + x + rx) / denominator;
        backleftPower = (y - x + rx) / denominator;
        frontrightPower = (y - x - rx) / denominator;
        backrightPower = (y + x - rx) / denominator;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        frontleftDrive.setPower(frontleftPower);
        backleftDrive.setPower(backleftPower);
        frontrightDrive.setPower(frontrightPower);
        backrightDrive.setPower(backrightPower);
    }

    void setTargetPosition (Pose2D target) {
        targetPosition = target;
    }
    void update () {
        odometry.update();
        double errorX = targetPosition.getX(DistanceUnit.INCH) - odometry.getPosition().getX(DistanceUnit.INCH);
        double errorY = targetPosition.getY(DistanceUnit.INCH) - odometry.getPosition().getY(DistanceUnit.INCH);
        double errorHeading = targetPosition.getHeading(AngleUnit.DEGREES) - odometry.getPosition().getHeading(AngleUnit.DEGREES);

        double powerX = errorX * kPX;
        double powerY = errorY * kPY;
        double powerHeading = errorHeading * kPHeading;
        drive(powerY, powerX, powerHeading);
    }
    public boolean isAtTargetPosition() {
        return errorX < XYThreshold && errorY < XYThreshold && errorHeading < headingThreshold;
    }

}
