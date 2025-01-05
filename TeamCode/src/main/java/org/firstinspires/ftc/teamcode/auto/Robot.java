package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.*;

public class Vector2D {
    private double i;  // X component
    private double j;  // Y component
    private double angle;  //in degrees

    // Constructor
    public Vector2D(double i, double j) {
        this.i = i;
        this.j = j;
        this.angle = Math.toDegrees(Math.atan2(j,i));
    }
    public Vector2D(double angle) {
        this.angle = angle;
        this.i = Math.cos(Math.toRadians(angle));
        this.j = Math.sin(Math.toRadians(angle));
    }

    public setVector(double i, double j) {
        this.i = i;
        this.j = j;
        this.angle = Math.toDegrees(Math.atan2(j,i));
    }

    public setAngle(double angle) {
        this.angle = angle;
        this.i = Math.cos(Math.toRadians(angle));
        this.j = Math.sin(Math.toRadians(angle));
    }
    public adjustAngle(double angle) {
        this.angle = this.angle - angle;
        this.i = Math.cos(Math.toRadians(angle));
        this.j = Math.sin(Math.toRadians(angle));
    }
    puiblic scaleVector(double scalar) {
        this.i = this.i * scalar;
        this.j = this.j * scalar;
    }



    // Getters for I and J components
    public double getI() {
        return i;
    }

    public double getJ() {
        return j;
    }

    public rotateVector() {

    }

    // To display the vector
    @Override

}


public Class Robot {
    private Motor backRightDrive;
    private Motor frontRightDrive;
    private Motor frontLeftDrive;
    private Motor backLeftDrive;
    private Servo outtakeAngle;
    private Servo outtakeClaw;

    private Servo intakeAngle;
    private Servo intakeClaw;

    private Servo intakeSlide1;
    private Servo intakeSlide2;

    private DcMotor elavator1;
    private DcMotor elavator2;

    private IMU imu;

    private DcMotor straight;
    private DcMotor sideways;
    private static final double WHEEL_DIAMETER = 48; // In milimeters
    private static final double TICKS_PER_REVOLUTION = 1120; /

    //private double current robotX; unable to reliably solve
    //private double current robotY;





    //this function will move the robot x distance and y distance, and make it face the direction
    public void driveRelative(double direction, double x, double y) {
        double distanceToTravel = Math.sqrt(x * x + y * y);
        double distanceTraveled = 0;
        double final MULTIPLIER = .01; //adjust this value to make the robot move less or more,
        // smaller values make it move farther
        double moveSpeed;
        double rotateSpeed;
        double currentYaw;
        double angleDifference;

        Vector2D toGo = new Vector2D(x, y);



        while (true) {
            currentYaw = imu.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if(distanceTraveled >= distanceToTravel) {
                toGo.setVector(x, y);
                toGo.adjustAngle(currentYaw);
                distanceTraveled += MULTIPLIER;
                moveSpeed = 0.3 * Math.sqrt(distanceToTravel*distanceToTravel - distanceTraveled*distanceTraveled);
                toGo.scaleVector(moveSpeed);
            } else {
                toGo.setVector(0, 0);
            }
            currentYaw = currentYaw + 180;

                angleDifference = Math.abs(currentYaw - direction);
                rX = 0;
                if (angleDifference < 359 && angleDifference > 1) {
                    rX += .3/4;
                }
            if (angleDifference < 350 && angleDifference > 10) {
                rX += .3/4;
            }
            if (angleDifference < 330 && angleDifference > 30) {
                rX += .3/4;
            }
            if (angleDifference < 300 && angleDifference > 60) {
                rX += .3/4;
            }

            if (direction-currentYaw < 0) {
                rX = rX*-1;
            }


            LeftFrontWheel = Speed + Strafe - Turn;
            RightFrontWheel = Speed - Strafe - Turn;
            LeftBackWheel = Speed - Strafe + Turn;
            RightBackWheel = Speed + Strafe - Turn;

            telemetry.addData("x", toGo.getI());
            telemetry.addData("y", toGo.getJ());
            telemetry.addData("rX", rX);
            telemetry.update();
            frontrightDrive.setPower(toGo.getJ() - toGo.getI() - rX); //double check these values
            frontleftDrive.setPower(toGo.getJ() + toGo.getI() + rX);
            backleftDrive.setPower(toGo.getJ() - toGo.getI() + rX);
            backrightDrive.setPower(toGo.getJ() + toGo.getI() - rX);


            if (rX == 0 && toGo.getI() == 0 && toGo.getJ() == 0) {
                break;
            }


        }


    }

public Robot(HardwareMap hardwareMap) {
    // Initialize the hardware devices
    frontRightDrive = new Motor("frontright", true, true);
    backRightDrive = new Motor("backright", false, true);
    frontLeftDrive = new Motor("frontleft", true, true);
    backLeftDrive = new Motor("backleft", false, true);

   // outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
    //outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

    //intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
    //intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

    //intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
    //intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

    //elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
    //elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
    imu.initialize(parameters);
    imu.resetYaw();

    straight = hardwareMap.get(DcMotor.class, "straight");
    sideways = hardwareMap.get(DcMotor.class, "sideways");
    straight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    sideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



}


}
