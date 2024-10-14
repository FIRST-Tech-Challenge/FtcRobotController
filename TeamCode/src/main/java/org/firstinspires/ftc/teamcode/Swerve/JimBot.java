package org.firstinspires.ftc.teamcode.Swerve;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

@TeleOp(name = "JimBot Swerve", group = "Swerve")
public class JimBot extends LinearOpMode {

    DcMotor FLMotor, BLMotor, BRMotor, FRMotor;

    Servo FLServo, BLServo, BRServo, FRServo;

    //in case builders are bad, is offset center for servo
    double FLServoOffSet = .01;
    double FRServoOffSet = .00;
    double BLServoOffSet = .01;
    double BRServoOffSet = .01;

    ElapsedTime turnTime = new ElapsedTime();

    GoBildaPinpointDriver odo;

    static double TRACKWIDTH = 14; //in inches
    static double WHEELBASE = 15; //in inches

    @Override
    public void runOpMode() {

        initRobot(); // Does all the robot stuff

        waitForStart();
        while (opModeIsActive()) {
            double speed = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angle = -gamepad1.right_stick_x;
            if (speed != 0)
                move(gamepad1.left_stick_x, speed);
            else if (angle != 0)
                rotate(angle);
            else {
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                BRMotor.setPower(0);
                FRMotor.setPower(0);
            }
            if (gamepad1.a)
                moveHome();
        }

    }


    public void initRobot() {

        //init odo
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(177.8, 50.8);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Maps the motor objects to the physical ports
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");

        // Sets the encoder mode
        FLMotor.setMode(RUN_USING_ENCODER);
        BLMotor.setMode(RUN_USING_ENCODER);
        BRMotor.setMode(RUN_USING_ENCODER);
        FRMotor.setMode(RUN_USING_ENCODER);

        // Sets what happens when no power is applied to the motors.
        // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
        FLMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);

        FLMotor.setDirection(REVERSE);
        BLMotor.setDirection(REVERSE);
        BRMotor.setDirection(FORWARD);
        FRMotor.setDirection(REVERSE);


        // Maps the servo objects to the physical ports
        FLServo = hardwareMap.get(Servo.class, "FLServo");
        BLServo = hardwareMap.get(Servo.class, "BLServo");
        BRServo = hardwareMap.get(Servo.class, "BRServo");
        FRServo = hardwareMap.get(Servo.class, "FRServo");

        // Sets the ends of the servos. Hover cursor over function for more info
        // Will need to be tuned later
        //FL Servo center .51
        //FR Servo center .55
        //BL Servo center .51
        //BR Servo center .51
        FLServo.scaleRange(FLServoOffSet, 1.0 + FLServoOffSet * 2);
        BLServo.scaleRange(BLServoOffSet, 1.0 + BLServoOffSet * 2);
        BRServo.scaleRange(BRServoOffSet, 1.0 + BRServoOffSet * 2);
        FRServo.scaleRange(FRServoOffSet, 1.0 + FRServoOffSet * 2);

        double[] test = cartesianToPolar(1, 0);

        FLServo.setPosition(0.50 + FLServoOffSet);
        FRServo.setPosition(0.50 + FRServoOffSet);
        BLServo.setPosition(0.51 + BLServoOffSet);
        BRServo.setPosition(0.51 + BRServoOffSet);
    }


    // Converts cartesian coordinates to polar
    // 0 = r
    // 1 = theta
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta


        return arrayToReturn;
    }


    /*
    Converts polar coordinates to cartesian
     0 = x
     1 = y
    */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }


    //oldH newH not used maybe later to have motors wait to turn wheel until Servo is in right position
    public void move(double heading, double power) {
        //double newH;
        /*
        /if(newH>=oldH-5 && newH <=oldH+5)
            oldH = newH;
         */

        heading = (heading + 1) / 2;

        FLServo.setPosition(heading + FLServoOffSet);
        BLServo.setPosition(heading + BLServoOffSet);
        BRServo.setPosition(heading + BRServoOffSet);
        FRServo.setPosition(heading + FRServoOffSet);

        //if(turnTime<)

        FLMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);
        FRMotor.setPower(power);
        //oldH = newH;
    }


    public void rotate(double angle) {

        //set wheels for rotation (Ben's robot has 2x gear ratio so .25/2 and .75/2)
        FLServo.setPosition(.25 + .125 / 2);
        BLServo.setPosition(.75 - .125/2);
        BRServo.setPosition(.25 + .125 / 2);
        FRServo.setPosition(.75 - .125/2);

        //turn motors to rotate robot
        FLMotor.setPower(-angle);
        BLMotor.setPower(-angle);
        BRMotor.setPower(angle);
        FRMotor.setPower(angle);

    }


    public void strafe(double power){

    }


    /*
    wheel angle is perpendicular to turn angle
    turn angle is inverse tan(get angle) of 7.5(half of wheel base length) / (turning distance/2)
    because it is radius of point we are trying to rotate around
    speed = -1 to 1
    turnRad = -1 to 1
    turnDir = LEFT or RIGHT
     */
    public void moveAndRotate(double speed, double turnAmount, turnDir dir) {

        double i_WheelAngle = Math.atan2(WHEELBASE, turnAmount - TRACKWIDTH / 2);
        double outsideAng = Math.atan2(WHEELBASE, turnAmount + TRACKWIDTH / 2);


    }


    //uses imu
    public void moveHome() {
        double orientation = odo.getHeading();
        telemetry.addData("Yaw angle", orientation);
        rotate(orientation);
    }


    public void moveToSpecimen() {

    }


    public void moveToStore() {

    }


    public enum turnDir {
        LEFT, RIGHT;
    }

}
