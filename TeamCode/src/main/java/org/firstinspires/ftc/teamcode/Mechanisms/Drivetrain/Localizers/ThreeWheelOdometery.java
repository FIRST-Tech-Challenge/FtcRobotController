package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.Utils.Utils;

@Config
public class ThreeWheelOdometery {
    public HardwareMap hardwareMap;
    public DcMotorEx leftEncoder;
    public DcMotorEx centerEncoder;
    public DcMotorEx rightEncoder;
    public static double ticksPerRotation = 2000;
    public static double radius = 0.944882; //[in]
    public double currentLeftRawPos = 0; //[ticks]
    public double currentCenterRawPos = 0; //[ticks]
    public double currentRightRawPos = 0; //[ticks]
    public double lastLeftRawPos = 0; //[ticks]
    public double lastCenterRawPos = 0; //[ticks]
    public double lastRightRawPos = 0; //[ticks]
    public double currentLeftVelocity = 0; //[ticks]
    public double currentCenterVelocity = 0; //[ticks]
    public double currentRightVelocity = 0; //[ticks]
    public static double trackWidth = 13.4311024; //[in]
    public static double forwardOffset = 7.87402; //[in]
    public ElapsedTime dt = new ElapsedTime(); //[s]
    public static double xMult = 1;
    public static double yMult = 1;


    // you should have a public static double xMult, public static double yMult
    public ThreeWheelOdometery(HardwareMap hardwareMap, DcMotorEx motor0, DcMotorEx motor1, DcMotorEx motor3) {
        this.hardwareMap = hardwareMap;
        this.leftEncoder = motor0;
        this.centerEncoder = motor1;
        this.rightEncoder = motor3;
        dt.reset();
    }

    // Change this to updateEncoderPositions
    // Change the phrase 'Val' everywhere to 'RawPos'
    public void updateEncoderPositions(){
        currentLeftRawPos = -leftEncoder.getCurrentPosition()*xMult;
        currentCenterRawPos = centerEncoder.getCurrentPosition()*yMult;
        currentRightRawPos = -rightEncoder.getCurrentPosition()*xMult;
    }

    // Write a updateEncoderVelocities
    // should just be encoderObject.getVelocity()

    public void updateEncoderVelocities(){
        currentLeftVelocity = -leftEncoder.getVelocity();
        currentCenterVelocity = centerEncoder.getVelocity();
        currentRightVelocity = -rightEncoder.getVelocity();
    }

    public void pushBackValues(){
        lastLeftRawPos=currentLeftRawPos;
        lastCenterRawPos=currentCenterRawPos;
        lastRightRawPos=currentRightRawPos;
    }

    // Change this a bit. Make the input a single double rawVal. For example,
    // now you would do convertToDistance(currentLeftVal - lastLeftVal) for the left position
    // and for velocity of the left encoder it would be convertToDistance(leftVelocity)
    public double convertToDistance(double rawVal){
        return 2*Math.PI*radius*(rawVal)/ticksPerRotation;
    }

    // Add an argument called lastState (SimpleMatrix)
    // Remove the theta argument and just access it through lastState.get(2, 0)
    public SimpleMatrix calculate(SimpleMatrix lastState){
        //Call BOTH updateEncoderPositions() and updateEncoderVelocities() HERE
        updateEncoderPositions();
        updateEncoderVelocities();

        double relativeChangeX = (convertToDistance(currentLeftRawPos - lastLeftRawPos) + convertToDistance(currentRightRawPos - lastRightRawPos))/2;
        double changeInHeading = (convertToDistance(currentLeftRawPos - lastLeftRawPos) - convertToDistance(currentRightRawPos - lastRightRawPos))/trackWidth;
        double relativeChangeY = convertToDistance(currentCenterRawPos - lastCenterRawPos)-forwardOffset*changeInHeading;

        // Do distance conversions for velocity
        // These will be encoder velocities with units of in/s
        // Use the same formula for relativeChangeX for vX, vY is simply going to be you horizontal encoder velocity

        double xVelocity = (convertToDistance(currentLeftVelocity) + convertToDistance(currentRightVelocity))/2;
        double angularVelocity = (convertToDistance(currentLeftVelocity) - convertToDistance(currentRightVelocity))/trackWidth;
        double yVelocity = convertToDistance(currentCenterVelocity)-forwardOffset*angularVelocity;


        SimpleMatrix deltaPoseBody = new SimpleMatrix(
                new double[][]{
                        new double[]{relativeChangeX},
                        new double[]{relativeChangeY},
                        new double[]{changeInHeading}
                }
        );
        SimpleMatrix poseExponentials = new SimpleMatrix(
                new double[][]{
                        new double[]{Math.sin(changeInHeading) / changeInHeading, (Math.cos(changeInHeading) - 1) / changeInHeading, 0},
                        new double[]{(Math.cos(changeInHeading) - 1) / changeInHeading, Math.sin(changeInHeading) / changeInHeading, 0},
                        new double[]{0, 0, 1}
                }
        );

        if (changeInHeading == 0) {
            poseExponentials = SimpleMatrix.identity(3);
        }
        SimpleMatrix deltaPose = Utils.rotateBodyToGlobal(poseExponentials, lastState.get(2, 0)).mult(deltaPoseBody);
        SimpleMatrix state = new SimpleMatrix(
                new double[][]{
                        new double[]{deltaPose.get(0,0) + lastState.get(0,0)},
                        new double[]{deltaPose.get(1,0) + lastState.get(1,0)},
                        new double[]{deltaPose.get(2,0)+lastState.get(2,0)},
                        new double[]{xVelocity},
                        new double[]{yVelocity},
                        new double[]{angularVelocity}
                }
        );
        // Create a new 6x1 state vector (SimpleMatrix)
        // For the first 3 elements add lastState + deltaPose
        // for the second 3 elements simply set them to be vX, vY and omega
        // Return the state

        pushBackValues();
        return state;
    }
}