package org.firstinspires.ftc.teamcode.Hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Usefuls.Math.T;

@Config
public class Drivetrain {
    private boolean auto = false;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx leftFront;
    ElapsedTime derivativeTimer = new ElapsedTime();
    private GoBildaPinpointDriver odo;
    public static double yOffset = 0; //tune
    public static double xOffset = 135; //tune

    private double zeroMoveAngle = 25;

    private double flPower, frPower, blPower, brPower;
    private double normalize;
    private Pose2d currentPose = new Pose2d(0,0,0);
    private Pose2d currentVelocity = new Pose2d(0,0,0);
    private double xRn, yRn, rRn;
    private double headingVelocity = 0;

    public static double P = 0, D = 0;
    public static double rP = 0, rD = 0;

    private BasicPID xController, yController, rController;
    private PIDCoefficients xyCoefficients, rCoefficients;
    //x and y must be the same because when you rotate 90 degrees, the x and y are swapped!
    private double xPower, yPower, rPower;
    private double xTarget, yTarget, rTarget;
    private Pose2d rawOutputs = new Pose2d();



    public Drivetrain(HardwareMap hardwareMap, ElapsedTime timer, Pose2d startPose) {
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

        xyCoefficients = new PIDCoefficients(P, 0, D);
        rCoefficients = new PIDCoefficients(P, 0, D);

        derivativeTimer = timer;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(xOffset, yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD); //maybe
        odo.resetPosAndIMU();
        odo.setPosition(startPose);
    }
    private double lastFlPower = 0;
    private double lastBlPower = 0;
    private double lastBrPower = 0;
    private double lastFrPower = 0;
    public void setPowers(double y, double x, double r){ //y moves forward, x strafes// , r rotates it
            normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

            flPower = (y+x-r);
            blPower = (y-x-r);
            brPower = (y+x+r);
            frPower = (y-x+r);

            flPower /= normalize;
            blPower /= normalize;
            brPower /= normalize;
            frPower /= normalize;
            // Only set power if the change is greater than 0.05
            if (Math.abs(flPower - lastFlPower) > 0.05) {
                leftFront.setPower(flPower);
                lastFlPower = flPower;
            }
            if (Math.abs(blPower - lastBlPower) > 0.05) {
                leftRear.setPower(blPower);
                lastBlPower = blPower;
            }
            if (Math.abs(brPower - lastBrPower) > 0.05) {
                rightRear.setPower(brPower);
                lastBrPower = brPower;
            }
            if (Math.abs(frPower - lastFrPower) > 0.05) {
                rightFront.setPower(frPower);
                lastFrPower = frPower;
            }
    }

    public void update(){

        //LOCATION DETAILS
        //only do this ONE TIME, access ONLY the variables "currentPose and currentVelocity" from now on.
        odo.update();
        currentPose = odo.getPosition();
        currentVelocity = odo.getVelocity();
        headingVelocity = currentVelocity.getHeading();

        if(auto == true){ //DONT UPDATE PID's IN TELEOP, JUST OUR CURRENT POSITION AND VELOCITY!
            //OUR CURRENT POSITION
            xRn = currentPose.getX();
            yRn = currentPose.getY();

            rRn = currentPose.getHeading();
            //CHECK THE UNITS OF HEADING AND WRITE IT IN A COMMENT!

            //PID controllers calculate motor powers based on how far away they are.
            xPower = xController.calculate(xTarget, xRn);
            yPower = yController.calculate(yTarget, yRn);
            rPower = rController.calculate(rTarget, rRn);

            rawOutputs = new Pose2d(xPower, yPower, rPower);

            //check the units of heading, they must be in radians for this to work
            //rotation matrix
            xPower = (xPower * T.cos(rRn) - yPower * T.sin(rRn));
            yPower = (xPower * T.sin(rRn) + yPower * T.cos(rRn));

//            //zeroMoveAngle is 25 for now
//            double errorScale = 1 - (Math.abs(rRn - rTarget) / zeroMoveAngle);
//            if (errorScale < 0) {
//                errorScale = 0;
//            }
//
//            xPower *= errorScale;
//            yPower *= errorScale;

//            setPowers(yPower, xPower, rPower);

//            setPowers(0, yPower, 0);
//            setPowers(xPower, 0, 0);
            setPowers(0, 0, rPower);



            //actually setting motor powers using the setPowers function!
            //negatives and stuff here
        }
    }

    public void autonomous(){
        auto = true;
    }

    public Pose2d getPose(){
        return currentPose;
    }
    public double getHeadingVelocity(){
        return headingVelocity;
    }
    public DcMotorEx getSlidesMotor(){
        return leftFront;
    }
    public DcMotorEx getArmMotor(){
        return leftRear;
    }
    public GoBildaPinpointDriver.DeviceStatus getStatus(){
        return odo.getDeviceStatus();
    }

    public void setTarget(Pose2d target){
        xTarget = target.getX();
        yTarget = target.getY();
        rTarget = target.getHeading();
    }
    //equivalent to the lineTo function
    //technically doesn't move in a line but neither did lineTo

    public void setZeroMoveAngle(double angle){
        zeroMoveAngle = angle;
    }

    public Pose2d Powers(){
        return new Pose2d(xPower, yPower, rPower);
    }
    public Pose2d PIDOutputs(){
        return rawOutputs;
    }

    public boolean isAtTarget(){
        if((Math.hypot(Math.abs(xTarget - xRn), Math.abs(yTarget - yRn)) < 2) && Math.abs(rTarget - rRn) < 2){
            return true;
        }else {
            return false;
        }
    }

}
