// Program created by: Danny and William
// Purpose: FTC Robot Software

package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder;

// Imports all of the necessary FTC libraries and code

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

// How to connect to robot procedure (see file called
// controlHubConnectionInstructions.md)

public class RobotInitialize {

    // Initialization Phase

    // Create servo variables
    public CRServo intake; // This is a special continuous rotation servo which allows it to act
    // like a motor
    public Servo parkingServo;

    public Servo pitch;
    //public Servo clawRoll;


    // Create the empty normal motor variables
    public DcMotorEx fLeft;
    public DcMotorEx bRight;
    public DcMotorEx fRight;
    public DcMotorEx bLeft;

    // Create the empty lift control variables
    public DcMotorEx liftExtender; //Extends the lift outwards and pulls it inwards
    public DcMotorEx liftPitch; //Makes the lift up and down on a vertical tilt (uses worm gear)

    // Create empty gyroscope variable and its settings
    public BHI260IMU gyroScope;
    BHI260IMU.Parameters settings;

    // A constructor that makes a new instance of
    // The orientation class called lastAngles
    Orientation lastAngles = new Orientation();
    double globalAngle;

    // Makes an instance of the class LinearOpMode called opMode
    LinearOpMode opMode;

    // Enables the class to be referenced in other classes such as the Autonomous Code
    // and the TeleOpCode_RobotCentric
    public RobotInitialize(LinearOpMode opMode, boolean isAuto) {
        this.opMode = opMode;
        initialize(isAuto);
    }

    // The main function that sets all of the hardware to different variables
    // The motors and the gyroscope are initialized here
    public void initialize(boolean isAuto) {
        // map the devices to the hardware map

        //Drivetrain motors
        fLeft = opMode.hardwareMap.get(DcMotorEx.class, "fleft");
        bRight = opMode.hardwareMap.get(DcMotorEx.class, "bright");
        fRight = opMode.hardwareMap.get(DcMotorEx.class, "fright");
        bLeft = opMode.hardwareMap.get(DcMotorEx.class, "bleft");

        // The front left and back right motors are reversed so all wheels go in the same direction
        // When a positive or negative value is used
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);



        // Resetting the encoders (distance measurement sensors)
        // and then start them again on program start
        // Repeat for all motors
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //without odom: bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Manipulator mechanisms

            //Lift motors

        liftExtender = opMode.hardwareMap.get(DcMotorEx.class, "liftExtender");

            //Initial conditions of the liftExtender MOTOR
        liftExtender.setVelocityPIDFCoefficients(2.67,2.05,0, 3.3);

        liftExtender.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isAuto) {
            liftExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Needs to not reset once teleop begins
            liftExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Needs to not reset once teleop begins
        }
            liftExtender.setZeroPowerBehavior(BRAKE);



        liftPitch = opMode.hardwareMap.get(DcMotorEx.class, "liftPitch");

            //Initial conditions of the liftPitch MOTOR
            //PIDFCoefficients pid = new PIDFCoefficients(1, 1, 1, 1); (This does nothing)
            //PIDF Coefficients for the liftPitch MOTOR
            liftPitch.setVelocityPIDFCoefficients(1,1,-2.5, 3);
            liftPitch.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isAuto) {
            liftPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Needs to not reset once teleop begins
            liftPitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Needs to not reset once teleop begins
        }
            liftPitch.setZeroPowerBehavior(BRAKE);

        //Manipulator Servos

            // Hang on submersible servos


        parkingServo = opMode.hardwareMap.get(Servo.class, "parkingServo");
        //hangR.setDirection(Servo.Direction.REVERSE);
        parkingServo.setPosition(1);

        //Continuous rotation Servo
        intake = opMode.hardwareMap.get(CRServo.class, "intake");
        //Initial conditions of the intake SERVO
        intake.setPower(0); // Off by default
        intake.setDirection(CRServo.Direction.REVERSE);
        //Regular Servos
        /*clawRoll = opMode.hardwareMap.get(Servo.class, "pitch");
        clawRoll.setPosition(0);*/
        pitch = opMode.hardwareMap.get(Servo.class, "roll");
        pitch.setPosition(0);

//        roll.setDirection(Servo.Direction.FORWARD);
//        roll.setPosition(0);

//        pitch.setDirection(Servo.)
        { //This was causing problems
            //claw.setPosition(0);
            //roll.setPosition(0);
            //pitch.setPosition(0);
        }



        // Initialize Gyroscope
        gyroScope = opMode.hardwareMap.get(BHI260IMU.class, "gyroScope");
        // Possibly change this to the following code comment
        RevHubOrientationOnRobot ori = new RevHubOrientationOnRobot(new Orientation(AxesReference
                .INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 90, 0, 0,
                0));

        // Way to set up the gyroscope
        // new REVHubOrientationOnRobot(REVHubOrientationOnRobot.LogoFacingDirection.<direction
        // here>, REVHubOrientationOnRobot.UsbFacingDirection.<direction here>));

        // Setting up the gyroscope settings
        settings = new BHI260IMU.Parameters(ori);
        gyroScope.initialize(settings);
        AngularVelocity angularVelocity = gyroScope.getRobotAngularVelocity(AngleUnit.DEGREES);
        YawPitchRollAngles orientation = gyroScope.getRobotYawPitchRollAngles();
        gyroScope.resetYaw();




        //odom sect
        final int wheelDiam = 48;

        //final int x = ;

    }


    // Gets the angle that the robot is currently facing in from the gyroscope
    private double getAngle() {
        Orientation angles = gyroScope.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        Orientation angles = gyroScope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360; // Changes the deltaAngle to be within a range of 0 to 360 degrees
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Sets the variable globalAngle to itself plus deltaAngle
        globalAngle += deltaAngle; // gA = gA + deltaAngle
        lastAngles = angles; // Sets lastAngles equal to
        return globalAngle;
    }

    double inchesToEncoderTicks;

    // This function makes the robot travel a relativeDistance specified by the parameter relativeDistance
    // This parameter is measured in encoder ticks; the other parameter, velocity, is
    // a decimal value that determines how fast the robot will go
    public void goStraight (int distance, double velocity) {
        // Travel a certain relativeDistance based on the absolute value
        // of the average robot encoder reading
        // The relativeDistance is current position plus or minus the value that is being moved
        // RELATIVE DISTANCE MEASUREMENT IN USE

        //Is supposed to be in inches
        //inchesToEncoderTicks = (distance*5267.65)/(12.56637061*13.7);
        //opMode.telemetry.addData("Receives a value in IN and converts to encoder ticks", inchesToEncoderTicks);

        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        // It only moves if the distance to the final location is greater than or equal to 10 encoder
        // ticks
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {

            //if the current position is before final position
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                setDrivetrainMotorVelocity(Math.abs(velocity));

                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
                //if current position is less than needed
            } else if (getPosStrafe() > relativeDistance) {
                // Backwards (- negative relativeDistance value)
                setDrivetrainMotorVelocity(-Math.abs(velocity));
                opMode.telemetry.addData("Encoder straight", getPosStrafe());
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMechanisms();
    }


    // Makes the robot strafe right by determining where the robot is currently
    // located and where it is trying to go it does not return anything and
    // has parameters of the distance it needs to travel (measured in encoder ticks)
    // and the velocity that it moves (measured in encoder ticks per second)

    public void strafeR(int distance, int velocity){
        int relativeDistance = distance + getPosStrafe();


        // Go forwards or backwards
        //if difference <10 then stop
        // 10 is the accuracy tolerance in 10 encoder ticks
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 14) {
            //if
            opMode.telemetry.addData("velocity", fLeft.getVelocity());

            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                //setMotorVelocity(Math.abs(velocity));
                fLeft.setVelocity(velocity);
                fRight.setVelocity(-velocity);
                bLeft.setVelocity(velocity);
                bRight.setVelocity(-velocity);

                opMode.telemetry.addData("position", getPosStrafe());
                opMode.telemetry.addData("relative distance", relativeDistance);
                opMode.telemetry.addData("old pos", getAverageEncoderValue());
                opMode.telemetry.addData("must be greater than 14", Math.abs(getPosStrafe() - relativeDistance));
                opMode.telemetry.update();
            } else{
                setDrivetrainMotorVelocity(0);
            }
        }
        setDrivetrainMotorVelocity(0);
        //stopMechanisms();
    }
    // Parameter of time that it is run in milliseconds
    public void extake(double time){
        double intakeRunTime = time;
        long x = System.currentTimeMillis();

        while (System.currentTimeMillis()-x <time && opMode.opModeIsActive()) { //Check this
            intake.setPower(-1);

            // opMode.telemetry.addData(System.currentTimeMillis());
//            if (intakeRunTime == 0){
//                break;
//            }
//            intakeRunTime = intakeRunTime + 1000;
        }
        intake.setPower(0);
    }
    public void intake(double time){
        double intakeRunTime = time;
        long x = System.currentTimeMillis();

        while (System.currentTimeMillis()-x <time && opMode.opModeIsActive()) { //Check this
            intake.setPower(1);

            // opMode.telemetry.addData(System.currentTimeMillis());
//            if (intakeRunTime == 0){
//                break;
//            }
//            intakeRunTime = intakeRunTime + 1000;
        }
        intake.setPower(0);
    }

    //needs to move counterclockwise to move up
    public void liftPitch(int position, double velocity){
        liftPitch.setTargetPosition(position);
        liftPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(liftPitch.getCurrentPosition()-position)>10 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("needs to be >10", Math.abs(liftPitch.getCurrentPosition()-position));
            opMode.telemetry.addData("position", liftPitch.getCurrentPosition());
            opMode.telemetry.update();
            if (liftPitch.getCurrentPosition()<position) {
                liftPitch.setPower(velocity);

            } else if (liftPitch.getCurrentPosition()>= position){
                liftPitch.setPower(-velocity);
            }
        }
    }

    // Extends the lift outwards and retracts it inwards
    public void liftExtender(int position, double velocity){
        liftExtender.setTargetPosition(position);
        liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(liftExtender.getCurrentPosition()-position)>10 && opMode.opModeIsActive()) {
            opMode.telemetry.addData("needs to be >10", Math.abs(liftExtender.getCurrentPosition()-position));
            opMode.telemetry.addData("position", liftExtender.getCurrentPosition());
            opMode.telemetry.update();
            if (liftExtender.getCurrentPosition()<position) {
                liftExtender.setPower(velocity);

            } else if (liftExtender.getCurrentPosition()>=position){
                liftExtender.setPower(-velocity);
            }
        }
    }

    // Makes the robot strafe left by determining where the robot is currently
    // located and where it is trying to go it does not return anything and
    // has parameters of the distance it needs to travel (measured in encoder ticks)
    // and the velocity that it moves (measured in encoder ticks per second)
    public void strafeL(int distance, int velocity){
        //relative distance is input value + pos
        int relativeDistance = distance + getPosStrafe();
        // Go forwards or backwards
        while (opMode.opModeIsActive() && Math.abs(getPosStrafe() - relativeDistance) >= 10) {
            if (getPosStrafe() < relativeDistance) {
                // Change into a function with a parameter, function name: setMotorVelocity
                // Forwards (+ positive relativeDistance value)
                //setMotorVelocity(Math.abs(velocity));
//
                Double[] motorVelocity = {bLeft.getVelocity(), fLeft.getVelocity(),
                        bRight.getVelocity(), fRight.getVelocity()};
                fLeft.setVelocity(-velocity);
                fRight.setVelocity(velocity);
                bLeft.setVelocity(-velocity);
                bRight.setVelocity(velocity);

                opMode.telemetry.addData("must be greated than 10", Math.abs(getPosStrafe() - (distance + getPosStrafe())));
                opMode.telemetry.addData("bleft", bLeft.getCurrentPosition());
                opMode.telemetry.addData("bright", bRight.getCurrentPosition());
                opMode.telemetry.addData("fright", fRight.getCurrentPosition());
                opMode.telemetry.addData("fleft", fLeft.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        stopMechanisms();
    }

    // This is the new turn function that includes setVelocity instead of setPower
    public void newTurnFunction(int degrees) {
        // When turning left, counterclockwise is a positive gyro value
        // When turning right, clockwise is a negative gyro value
        // ABSOLUTE POSITIONING IN USE (will go to exact values)

        while (opMode.opModeIsActive() && Math.abs(degrees - getAngle()) >= 0.2) {
            opMode.telemetry.addData("Encoder turn:", gyroScope.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            opMode.telemetry.addData("Gyroscope", getAngle());
            opMode.telemetry.update();

            if (degrees > getAngle()) {
                // Turning left (positive gyro value)
                fLeft.setVelocity(-300);
                bLeft.setVelocity(300);
                fRight.setVelocity(300);
                bRight.setVelocity(-300);
            }
            else if (degrees < getAngle()) {
                // Turning right (negative gyro value)
                fLeft.setVelocity(300);
                bLeft.setVelocity(-300);
                fRight.setVelocity(-300);
                bRight.setVelocity(300);
            }
        }
        stopMechanisms();
    }

//    public void intakeToggle(double power) {
//    intake.setPower(power);
//    }

    public void moveRoll(double position, double velocity) {
        //clawRoll.setPosition(position);
    }
    public void movePitch(double position, double velocity) {
        //pitch.setPosition(position);
    }

    //gets average magnitudes because motors are going in different directions
    public int getPosStrafe() {
        return ((Math.abs(fRight.getCurrentPosition())+Math.abs(bRight.getCurrentPosition())+Math.abs(fLeft.getCurrentPosition())+ Math.abs(bLeft.getCurrentPosition()))/4);
    }

    //
//    public int getPosStrafeL() {
//        return ((-fright.getCurrentPosition()+ bright.getCurrentPosition()+fleft.getCurrentPosition()- bleft.getCurrentPosition())/4);
//    }
    // Calculates the average encoder value
    // It takes the left and right motor encoder locations, then averages them
    public int getAverageEncoderValue() {
        return ((getLeftSideEncoderValues() + getRightSideEncoderValues()) / 2);
    }

    // Calculates the average left side encoder values
    // It takes the left side encoder location of the left side motors and averages them
    public int getLeftSideEncoderValues() {
        return ((fLeft.getCurrentPosition() + bLeft.getCurrentPosition())/2);
    }

    // Calculates the average right side encoder values
    // It takes the right side encoder location of the right side motors and averages them
    public int getRightSideEncoderValues() {
        return ((fRight.getCurrentPosition() + bRight.getCurrentPosition())/2);
    }

    // This sets the movement of the motors to be constant
    // The back wheels are set to negative velocity so the
    // robot goes forward when the velocity value is positive and
    // vice versa for going backwards
    public void setDrivetrainMotorVelocity(double velocity) {
        // Drivetrain motors
        fLeft.setVelocity(velocity);
        fRight.setVelocity(velocity);
        bLeft.setVelocity(-velocity);
        bRight.setVelocity(-velocity);
    }

    public void setMechanismVelocity(double velocity){
        // Drivetrain motors
        fLeft.setVelocity(velocity);
        fRight.setVelocity(velocity);
        bLeft.setVelocity(-velocity);
        bRight.setVelocity(-velocity);

        // Lift motors
        liftExtender.setVelocity(velocity);
        liftPitch.setVelocity(velocity);


        // Regular servos
        //clawRoll.(); // Try to find way to temporarily disable servos
        //pitch.(); // Try to find way to temporarily disable servos
    }

    // Stops the motors by setting the velocity to 0
    public void stopMechanisms() {
        setMechanismVelocity(0);
    }
}


