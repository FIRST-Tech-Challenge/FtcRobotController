package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.*;
import java.util.concurrent.TimeUnit;

// Related to vision
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    Claw claw = new Claw();
    SwingArm swingArm = new SwingArm();
    Chassis chassis = new Chassis();
    VSlider verticalSlider = new VSlider();

    public double swingArmHoldingPower = 1;


    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    public static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AWtcstb/////AAABmfYaB2Q4dURcmKS8qV2asrhnGIuQxM/ioq6TnYqZseP/c52ZaYTjs4/2xhW/91XEaX7c3aw74P3kGZybIaXued3nGShb7oNQyRkVePnFYbabnU/G8em37JQrH309U1zOYtM3bEhRej91Sq6cf6yLjiSXJ+DxxLtSgWvO5f+wM3Wny8MbGUpVSiogYnI7UxEz8OY88d+hgal9u3GhhISdnNucsL+fRAE8mKwT1jGDgUVE1uAJoZFvo95AJWS2Yhdq/N/HpxEH3sBXEm99ci+mdQsl0m96PMCDfV5RgWBjhLbBEIJyQ/xKAbw5Yfr/AKCeB86WDPhR3+Mr8BUvsrycZA6FDJnN5sZZwTg0ZE22+gFL";
    public VuforiaLocalizer vuforia; //vuforia object stored in vision class.
    public TFObjectDetector tfod; //tfod object stored in vision class.

    /* local OpMode members. */
    //Init hardware map
    HardwareMap hwMap = null;


    public ElapsedTime period = new ElapsedTime();
    //tells you how long the robot has run for


    //
    public void Robot() {

    }

    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public  void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        // this.tfod.loadModelFromFile(tfodPath, LABELS);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);


        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.1, 16.0 / 9.0);
        }
    }


    public void Drive(double speed, int distance) {

        runtime.reset();
        timeout_ms = 10000;

        chassis.robotY += distance;
        chassis.Location[1] = chassis.robotY;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = chassis.FLMotor.getCurrentPosition();
        int FRPos = chassis.FRMotor.getCurrentPosition();
        int BLPos = chassis.BLMotor.getCurrentPosition();
        int BRPos = chassis.BRMotor.getCurrentPosition();

        targetFR = FRPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetBL = BLPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);

        //Set motor targets
        chassis.FLMotor.setTargetPosition(targetFL);
        chassis.BLMotor.setTargetPosition(targetBL);
        chassis.FRMotor.setTargetPosition(targetFR);
        chassis.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        chassis.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        chassis.FLMotor.setPower(speed);
        chassis.FRMotor.setPower(speed);
        chassis.BLMotor.setPower(speed);
        chassis.BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (chassis.FLMotor.isBusy() && chassis.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void Strafe(double speed, int distance) {

        chassis.robotX += distance;

        runtime.reset();
        timeout_ms = 10000;

        chassis.robotX += distance;
        chassis.Location[0] = chassis.robotX;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = chassis.FLMotor.getCurrentPosition();
        int FRPos = chassis.FRMotor.getCurrentPosition();
        int BLPos = chassis.BLMotor.getCurrentPosition();
        int BRPos = chassis.BRMotor.getCurrentPosition();

        targetFR = FRPos - (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * chassis.COUNTS_PER_CM_Hex);
        targetBL = BLPos - (int) (distance * chassis.COUNTS_PER_CM_Hex);

        //Set motor targets
        chassis.FLMotor.setTargetPosition(targetFL);
        chassis.BLMotor.setTargetPosition(targetBL);
        chassis.FRMotor.setTargetPosition(targetFR);
        chassis.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        chassis.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chassis.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        chassis.FLMotor.setPower(speed);
        chassis.FRMotor.setPower(speed);
        chassis.BLMotor.setPower(speed);
        chassis.BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (chassis.FLMotor.isBusy() && chassis.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void stopDriveMotors(){
        // Stop all motion;
        chassis.FLMotor.setPower(0);
        chassis.FRMotor.setPower(0);
        chassis.BLMotor.setPower(0);
        chassis.BRMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        chassis.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveToPosition(double Speed, int posX, int posY, boolean forwardFirst) {
        if(forwardFirst) {
            this.Drive(Speed, -posY);
            this.Strafe(Speed, -posX);
        }
        else{
            this.Strafe(Speed, -posX);
            this.Drive(Speed, -posY);
        };
        System.out.println(Arrays.toString(chassis.Location));
    }

    public float modAngle(float angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    //Turns the robot
    public void turnRobotToAngle(float endAngle) {
        org.firstinspires.ftc.robotcore.external.navigation.Orientation angle;
        angle = chassis.imu.getAngularOrientation();

        float angleStart = modAngle(angle.firstAngle);
        float angleEnd = modAngle(endAngle);
        float angleCurrent = angleStart;
        float direction = 0;

        if(modAngle((angleEnd - angleCurrent)) >= 180) {
            //Go CW
            direction = -1;
        } else if (modAngle((angleEnd - angleCurrent)) <= 180) {
            //Go CCW
            direction = 1;
        }

        double pwr = -0.5;


        while (Math.abs(angleCurrent - angleEnd) > 5) {
            chassis.FLMotor.setPower(-pwr * direction);
            chassis.FRMotor.setPower(pwr * direction);
            chassis.BLMotor.setPower(-pwr * direction);
            chassis.BRMotor.setPower(pwr * direction);
            angle = chassis.imu.getAngularOrientation();
            angleCurrent = modAngle(angle.firstAngle);

        }
        stopDriveMotors();
    }

    public void MoveSlider(double speed, int Position, int timeout) {
        timeout_ms = timeout;

        runtime.reset();

//        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlider.vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlider.vSlider.setTargetPosition(Position);
        //Set the power of the motor.
        verticalSlider.vSlider.setPower(speed);
        //Run to position.
        verticalSlider.vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.milliseconds() < timeout_ms) && (verticalSlider.vSlider.isBusy())) {
        }
        verticalSlider.vSlider.setPower(0);
    }


    public void SwingArm(boolean updown){
        timeout_ms = 5000;
        runtime.reset();
        double speed;
        int Position;
        double holdingPower;

        if(updown){ // Swing arm up.
            speed = 1;
            Position = 75;
            holdingPower = 1;
        }
        else{ // Swing arm down.
             speed = 0.5;
             Position = 20;
             holdingPower =0;
        }

        swingArm.swingArm.setTargetPosition(Position);
        //Set the power of the motor.
        swingArm.swingArm.setPower(speed);
        //set the mode to go to the target position
        swingArm.swingArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while ((runtime.milliseconds() < timeout_ms) && (swingArm.swingArm.isBusy())) {

        }
        swingArm.swingArm.setPower(holdingPower);

    }

    public void SwingArmToPosition(double speed, int Position, double holdingPower) {
        timeout_ms = 5000;
        runtime.reset();

        if(Position>50){//Swing up
            swingArm.swingArm.setPower(0.1);
        }
        if(Position<50){ // Swing down
            swingArm.swingArm.setPower(-0.1);
        }

    }

    public void Park(int location) {
        if (location == 1) {
            claw.claw.setPosition(0);
            this.DriveToPosition(0.3, -75, 75, true);
        }

        if (location == 2) {
            claw.claw.setPosition(0);
            this.DriveToPosition(0.3, 0, 75, true);
        }

        if (location == 3) {
            claw.claw.setPosition(0);
            this.DriveToPosition(0.3, 75, 75, true);

        }
    }

    public void initArmClaw(){
        claw.claw.setPosition(1);
        SwingArmToPosition(1,75, swingArmHoldingPower);
        claw.claw.setPosition(0);

    }

    public void deliverPreLoad(boolean LR) {
        /** First swing the arm up and go to the pole. **/
        //Close claw, the arm is already up.
        claw.claw.setPosition(1);
        swingArm.swingArm.setPower(swingArmHoldingPower);

        timeout_ms = 300;
        runtime.reset();
        while(runtime.milliseconds() < timeout_ms){

        }

        DriveToPosition(0.5, 0, 77, true);


        //Drive to the pole
        if(LR) { // True = Left
            turnRobotToAngle(315);
        }
        else{
            turnRobotToAngle(45);
        }

        DriveToPosition(0.5, 0, 5, true);

        /** Next, move the slider to the right height, swing the arm down, drop the cone, swing the arm back up, and lower the slider. **/
        //Moves the slider to the correct height
        MoveSlider(1, 1000, 1550);


//        //Swings the arm down
         SwingArmToPosition(-1, 20, 0);
         //SwingArm(false);

        // Lower the slider a little bit to catch the cone in pole.
        MoveSlider(1, -500, 100);

        //Open and close the claw to drop the cone
        claw.claw.setPosition(0);
        timeout_ms = 1500;
        runtime.reset();
        while ((runtime.milliseconds() < timeout_ms)) {
        }
        claw.claw.setPosition(1);

        //Raises slider a little bit to not get caught on the pole
        MoveSlider(1, 1000, 500);
        //Swings the arm back up
        SwingArmToPosition(1, 70, swingArmHoldingPower);
        //lower the slider
        MoveSlider(-1, -1000, 1200);

    }

    public void ParkFromMedium(int location){
        turnRobotToAngle(360);
        switch(location){
            case 1:
                DriveToPosition(0.5, -70, -5, true);
                break;
            case 2:
                DriveToPosition(0.5, 0, -5, true);
                break;
            case 3:
                DriveToPosition(0.5, 70, -5, true);
                break;
        }
        turnRobotToAngle(175);
        DriveToPosition(0.5, 0, -15, true);


    }



    public void CycleCone(boolean LR){
        if(LR){

        }
        /** First go to the stack of cones and grab a cone **/
        //Open the claw and swing the arm down
        claw.claw.setPosition(0);
        SwingArmToPosition(1,20, 0);
        //Drive forward slightly
        DriveToPosition(0.6, 0, 25, true);
        //close the claw and grab onto the cone
        claw.claw.setPosition(1);
        /** Now drive to the medium pole **/
        //Drive to the pole and face it
        DriveToPosition(0.7,0,-60, true);
        turnRobotToAngle(210);
        stopDriveMotors();
        /** Now deliver the cone **/
        //Move the slider to the right height and swing down
        MoveSlider(0.6, 1000,2400);
        SwingArmToPosition(1, 20, 0);
        //Open and close claw
        claw.claw.setPosition(1);
        // sleep(500); // TODO: Replace with while loop timer.
        claw.claw.setPosition(0);
        //swing arm back up
        SwingArmToPosition(1, 70, swingArmHoldingPower);
        //lower slider
        MoveSlider(0.6, 0,1200);
        //Moves back to the stack
        turnRobotToAngle(90);
        stopDriveMotors();
        DriveToPosition(0.7,0,60, true);
    }

    public void closeClaw() {
        claw.claw.setPosition(claw.CLOSE);
    }

    public void openClaw() {
        claw.claw.setPosition(claw.OPEN);
    }

    public int getVSliderPos() {
        int VSliderPosition = verticalSlider.vSlider.getCurrentPosition();
        return VSliderPosition;
    }
    public int getSwingArmPos() {
        int SwingArmPosition = swingArm.swingArm.getCurrentPosition();
        return SwingArmPosition;
    }
    public double getClawPos() {
        double position = claw.claw.getPosition();
        return position;
    }
    public int getFLMotorPos() {
        int position = chassis.FLMotor.getCurrentPosition();
        return position;
    }
    public int getFRMotorPos() {
        int position = chassis.FRMotor.getCurrentPosition();
        return position;
    }
    public int getBLMotorPos() {
        int position = chassis.BLMotor.getCurrentPosition();
        return position;
    }
    public int getBRMotorPos() {
        int position = chassis.BRMotor.getCurrentPosition();
        return position;
    }

    public void setVSliderPower(double power){
        verticalSlider.vSlider.setPower(power);
    }

}