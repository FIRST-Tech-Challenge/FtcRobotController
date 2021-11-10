package org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.Utills.Executable;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;

public class AutonomousDriveSystem extends OdometryDrivetrain {
    OdometryPodServos pod;
    Thread positionThread;

    public AutonomousDriveSystem(HardwareMap hardwareMap, String front_right, String front_left, String back_right, String back_left,
                                 String horizontalEncoderName, String leftEncoderName, String rightEncoderName,String horizontalServo,
                                 String leftVerticalServo,String rightVerticalServo,
                                 Executable<Boolean> isStopRequested, Executable<Boolean> isOpModeActive,Telemetry telemetry) {


        super(); //initializes the underlying OdometryDrivetrain and BasicDrivetrain to null

        //Initializes dc motor array where the name of motorObjects[x] is motorNames[x]
        //for example motorNames[1] could be "Motor 1", thus the DcMotor object motorObjects[1]
        //would have been created from the string "Motor 1"
        String[] motorNames = {front_left, front_right, back_left, back_right};
        DcMotor[] motorObjects =  {
            hardwareMap.dcMotor.get(front_left),
            hardwareMap.dcMotor.get(front_right),
            hardwareMap.dcMotor.get(back_left),
            hardwareMap.dcMotor.get(back_right)

        };
        //loops over the array of motorObjects and initializes them
        for (int i = 0; i < motorNames.length; i++) {
            motorObjects[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorObjects[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*Allocates a DcMotor array of length three where, once filled out, the name of encoderObjects[x]
            will be the name of encoderObjects[x]
         */
        String[] encoderNames = {horizontalEncoderName, leftEncoderName, rightEncoderName};
        DcMotor[] encoderObjects = new DcMotor[3];

        /*
        iterates through each encoder name provided
         */
        for (int y = 0; y < encoderNames.length; y++) {

        /*
        iterates through each motor name provided and checks it against the current encoder name in the outer loop
         */
            for (int i = 0; i < motorNames.length; i++) {
                //if the current encoder name matches the motor name, the motor objects them selves are set equal
                if (encoderNames[y] == motorNames[i]) {
                    encoderObjects[y] = motorObjects[i];
                    break;
                }

            }
            //if it loops over all provided drive motor names and does not find a match, it initializes the encoder accordingly
            if (encoderObjects[y] == null) {
                encoderObjects[y] = hardwareMap.dcMotor.get(encoderNames[y]);
                encoderObjects[y].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderObjects[y].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        //DcMotor front_right, DcMotor front_left, DcMotor back_right, DcMotor back_left
        this.front_right = motorObjects[1];
        this.front_left = motorObjects[0];
        this.back_right = motorObjects[3];
        this.back_left = motorObjects[2];

        this.odometry = new OdometryGlobalCoordinatePosition(encoderObjects[1], encoderObjects[2], encoderObjects[0], 25);

        this.reinitializeMotors();
        this._isStopRequested = isStopRequested;
        this._opModeIsActive = isOpModeActive;


        pod = new OdometryPodServos(hardwareMap, rightVerticalServo, leftVerticalServo, horizontalServo);

    }

    public void setPosition(double x, double y, double angle){
        odometry.setPosition(x,y,angle);
    }

    public void raiseOdometryPods() {
        pod.raise();
    }

    public void lowerOdometryPods() {
        pod.lower();
    }

    public void reverseLeftOdometryEncoder(){
        this.odometry.reverseLeftEncoder();
    }

    public void reverseRightOdometryEncoder(){
        this.odometry.reverseRightEncoder();
    }

    public void reverseHorizontalOdometryEncoder(){
        this.odometry.reverseNormalEncoder();
    }

    public void start() {
        positionThread = new Thread(odometry);
        positionThread.start();
    }

    public double returnXCoordinate() {
        return odometry.returnRelativeXPosition();
    }

    public double returnYCoordinate() {
        return odometry.returnRelativeYPosition();
    }

    public double returnOrientation() {
        return odometry.returnOrientation();
    }

}

