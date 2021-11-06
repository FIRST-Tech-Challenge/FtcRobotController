package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.Executable;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.OdometryMovement;

public class AutonomousDriveSystem {
    OdometryPodServos pod;

    OdometryGlobalCoordinatePosition odometry;
    OdometryMovement odMovement;
    Thread positionThread;

    Executable<Boolean> isStopRequested;
    Executable<Boolean> isOpModeActive;
    Telemetry telemetry;




    public AutonomousDriveSystem(HardwareMap hardwareMap, String front_right, String front_left, String back_right, String back_left, String horizontalEncoderName, String leftEncoderName, String rightEncoderName,String horizontalServo, String leftVerticalServo,String rightVerticalServo, Executable<Boolean> isStopRequested, Executable<Boolean> isOpModeActive) {
        this.isStopRequested = isStopRequested;
        this.isOpModeActive = isOpModeActive;

        //Initializes dc motor array where the name of motorObjects[x] is motorNames[x]
        String[] motorNames = {front_left, front_right, back_left, back_right};
        DcMotor[] motorObjects = new DcMotor[4];
        for (int i = 0; i < motorNames.length; i++) {
            motorObjects[i] = hardwareMap.dcMotor.get(motorNames[i]);
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

        //
        odometry = new OdometryGlobalCoordinatePosition(encoderObjects[1], encoderObjects[2], encoderObjects[0], 1892.3724283364, 25);

        odMovement = new OdometryMovement(motorObjects[1],motorObjects[0],motorObjects[3],motorObjects[2], telemetry, odometry, isStopRequested, isOpModeActive);
        odMovement.reinitializeMotors();


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

    public void start() {
        positionThread = new Thread(odometry);
        positionThread.start();
    }

    public void stop() {
        odometry.stop();
    }

    public void moveToPosition(double x, double y, double tolerance) throws InterruptedException {
        odMovement.moveToPosition(x, y, tolerance);
    }

    public void strafeAtAngle(double power, double angle){
        odMovement.strafeAtAngle(45,0.1);
    }

    public double returnXCoordinate() {
        return odometry.returnXCoordinate();
    }

    public double returnYCoordinate() {
        return odometry.returnYCoordinate();
    }

    public double returnOrientation() {
        return odometry.returnOrientation();
    }

}

