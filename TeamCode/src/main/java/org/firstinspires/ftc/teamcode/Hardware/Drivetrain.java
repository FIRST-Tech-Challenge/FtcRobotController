package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;

@Config
public class Drivetrain {
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx leftFront;
    ElapsedTime derivativeTimer = new ElapsedTime();
    private GoBildaPinpointDriver odo;
    public static double yOffset = 100; //tune
    public static double xOffset = 84; //tune

    private double flPower, frPower, blPower, brPower;
    private double normalize;
    private Pose2d currentPose = new Pose2d(0,0,0);
    private double xRn, yRn, rRn;


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
        derivativeTimer = timer;
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(xOffset, yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //maybe
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
        odo.update();
        currentPose = odo.getPosition();
        xRn = currentPose.getX();
        yRn = currentPose.getY();
        rRn = currentPose.getHeading();
    }

    public Pose2d getPose(){
        return currentPose;
    }
    public GoBildaPinpointDriver.DeviceStatus getStatus(){
        return odo.getDeviceStatus();
    }

}
