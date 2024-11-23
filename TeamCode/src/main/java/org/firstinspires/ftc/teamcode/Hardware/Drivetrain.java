package org.firstinspires.ftc.teamcode.Hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Usefuls.Math.T;

@Config
public class Drivetrain {
    public final double yOffset = 0;
    public final double xOffset = 135;

    public static double zeroMoveAngle = 45;
    public static double P = 0.15, D = 4.5;
    public static double rP = 2.2, rD = 27;
    private boolean auto = false;

    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx leftFront;
    private final GoBildaPinpointDriver odo;


    private Pose2d startPose;
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    private double xRn, yRn, rRn;
    private double headingVelocity = 0;

    private final BasicPID xController;
    private final BasicPID yController;
    private final BasicPID rController;
    //x and y must be the same because when you rotate 90 degrees, the x and y are swapped!

    private double xPower, yPower, rPower;
    private double xTarget, yTarget, rTarget;

    private Pose2d rawOutputs = new Pose2d();

    private double lastFlPower = 0;
    private double lastBlPower = 0;
    private double lastBrPower = 0;
    private double lastFrPower = 0;
    public static double maxPower=1;

    public Drivetrain(HardwareMap hardwareMap, Pose2d startPose) {
        leftFront = hardwareMap.get(DcMotorEx.class, "bl");
        leftRear = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDCoefficients xyCoefficients = new PIDCoefficients(P, 0, D);
        PIDCoefficients rCoefficients = new PIDCoefficients(rP, 0, rD);

        xController = new BasicPID(xyCoefficients);
        yController = new BasicPID(xyCoefficients);
        rController = new BasicPID(rCoefficients);


        xTarget = startPose.getX();
        yTarget = startPose.getY();
        rTarget = startPose.getHeading();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(xOffset, yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD); //maybe
        odo.resetPosAndIMU();
        odo.setPosition(startPose);
    }

    public void setPowers(double y, double x, double r) { //y moves forward, x strafes// , r rotates it
        double normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

        double flPower = (y + x - r);
        double blPower = (y - x - r);
        double brPower = (y + x + r);
        double frPower = (y - x + r);

        flPower /= normalize;
        blPower /= normalize;
        brPower /= normalize;
        frPower /= normalize;

        // Set small powers to 0 if below .05
        if (Math.abs(flPower) < 0.05) {
            flPower = 0;
        }
        if (Math.abs(blPower) < 0.05) {
            blPower = 0;
        }
        if (Math.abs(brPower) < 0.05) {
            brPower = 0;
        }
        if (Math.abs(frPower) < 0.05) {
            frPower = 0;
        }

        // Only set power if the change is greater than 0.05
        if (Math.abs(flPower - lastFlPower) > 0.05) {
            leftFront.setPower(flPower);
            lastFlPower = flPower*maxPower;
        }
        if (Math.abs(blPower - lastBlPower) > 0.05) {
            leftRear.setPower(blPower);
            lastBlPower = blPower*maxPower;
        }
        if (Math.abs(brPower - lastBrPower) > 0.05) {
            rightRear.setPower(brPower);
            lastBrPower = brPower*maxPower;
        }
        if (Math.abs(frPower - lastFrPower) > 0.05) {
            rightFront.setPower(frPower);
            lastFrPower = frPower*maxPower;
        }
    }

    public void update() {

        //LOCATION DETAILS
        //only do this ONE TIME, access ONLY the variables "currentPose and currentVelocity" from now on.
        odo.update();
        currentPose = odo.getPosition();
        //CONVERT TO RR COORDINATES
        xRn = -currentPose.getY();
        yRn = currentPose.getX();

        rRn = currentPose.getHeading();
        //CHECK THE UNITS OF HEADING AND WRITE IT IN A COMMENT!
        currentPose = new Pose2d(xRn, yRn, rRn);
        Pose2d currentVelocity = odo.getVelocity();
        headingVelocity = currentVelocity.getHeading();

        if (auto) { //DONT UPDATE PID's IN TELEOP, JUST OUR CURRENT POSITION AND VELOCITY!
            //OUR CURRENT POSITION


            //PID controllers calculate motor powers based on how far away they are.
            double xOut = -xController.calculate(xTarget, xRn);
            double yOut = -yController.calculate(yTarget, yRn);
            rPower = rController.calculate(rTarget, rRn);

            rawOutputs = new Pose2d(xOut, yOut, rPower);

            //check the units of heading, they must be in radians for this to work
            //rotation matrix
            xPower = (xOut * T.cos(rRn) + yOut * T.sin(rRn));
            yPower = (xOut * T.sin(rRn) - yOut * T.cos(rRn));


            //zeroMoveAngle is 25 for now
            double errorScale = 1 - (Math.abs(rRn - rTarget) / Math.toRadians(zeroMoveAngle));
            if (errorScale < 0) {
                errorScale = 0;
            }

            xPower *= errorScale;
            yPower *= errorScale;



            setPowers(yPower, -xPower, rPower);
            //actually setting motor powers using the setPowers function!
        }
    }

    public void autonomous() {
        auto = true;
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public double getHeadingVelocity() {
        return headingVelocity;
    }

    public DcMotorEx getSlidesMotor() {
        return leftFront;
    }

    public DcMotorEx getArmMotor() {
        return leftRear;
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return odo.getDeviceStatus();
    }

    public void setTarget(Pose2d target) {
        xTarget = target.getX();
        yTarget = target.getY();
        rTarget = Math.toRadians(target.getHeading());
    }

    public void setZeroMoveAngle(double angle) {
        zeroMoveAngle = angle;
    }

    public Pose2d Powers() {
        return new Pose2d(xPower, yPower, rPower);
    }

    public Pose2d PIDOutputs() {
        return rawOutputs;
    }

    public boolean isAtTarget() {
        return (Math.hypot(Math.abs(xTarget - xRn), Math.abs(yTarget - yRn)) < 1) && Math.abs(rTarget - rRn) < 2;
    }

    public double distanceToPoint(Pose2d point){
        return Math.hypot(Math.abs(point.getX() - xRn), Math.abs(point.getY() - yRn));
    }

    public void setStartPostion(Pose2d startpose){
        odo.setPosition(startpose);
    }
    public void setMaxPower(double power){
        maxPower = power;
    }
}
