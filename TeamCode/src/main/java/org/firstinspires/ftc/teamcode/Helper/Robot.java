package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Related to IMU

import java.util.*;

// Related to vision
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Robot {
    /*
    Properties that describe hardware.
     */
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    public Claw claw = new Claw();
    public Arm arm = new Arm();
    public Chassis chassis = new Chassis();
    public VSlider vSlider = new VSlider();

    public double armHoldingPower = 1;


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



    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
        claw.init(hwMap);
        chassis.init(hwMap);
        arm.init(hwMap);
        vSlider.init(hwMap);


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
        // chassis.tfod.loadModelFromFile(tfodPath, LABELS);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);


        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.1, 16.0 / 9.0);
        }
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
        chassis.stopDriveMotors();
    }




    public void Park(int location) {
        if (location == 1) {
            claw.servo.setPosition(0);
            chassis.DriveToPosition(0.3, -75, 75, true);
        }

        if (location == 2) {
            claw.servo.setPosition(0);
            chassis.DriveToPosition(0.3, 0, 75, true);
        }

        if (location == 3) {
            claw.servo.setPosition(0);
            chassis.DriveToPosition(0.3, 75, 75, true);

        }
    }

    public void initArmClaw(){
        claw.close();
        arm.swingUp();
        claw.open();

    }

    public void deliverPreLoad(boolean LR) {
        /** First swing the arm up and go to the pole. **/
        //Close claw, the arm is already up.
        claw.servo.setPosition(1);
        arm.motor.setPower(armHoldingPower);

        timeout_ms = 300;
        runtime.reset();
        while(runtime.milliseconds() < timeout_ms){

        }

        chassis.DriveToPosition(0.5, 0, 77, true);


        //Drive to the pole
        if(LR) { // True = Left
            turnRobotToAngle(315);
        }
        else{
            turnRobotToAngle(45);
        }

        chassis.DriveToPosition(0.5, 0, 5, true);

        /** Next, move the slider to the right height, swing the arm down, drop the cone, swing the arm back up, and lower the slider. **/
        //Moves the slider to the correct height
       vSlider.MoveSlider(1, 1000, 1550);


//        //Swings the arm down
         arm.swingDown();
         //Arm(false);

        // Lower the slider a little bit to catch the cone in pole.
       vSlider.MoveSlider(1, -500, 100);

        //Open and close the claw to drop the cone
        claw.open();
        timeout_ms = 1500;
        runtime.reset();
        while ((runtime.milliseconds() < timeout_ms)) {
        }
        claw.close();

        //Raises slider a little bit to not get caught on the pole
       vSlider.MoveSlider(1, 1000, 500);
        //Swings the arm back up
        arm.swingUp();
        //lower the slider
       vSlider.MoveSlider(-1, -1000, 1200);

    }

    public void ParkFromMedium(int location){
        turnRobotToAngle(360);
        switch(location){
            case 1:
                chassis.DriveToPosition(0.5, -70, -5, true);
                break;
            case 2:
                chassis.DriveToPosition(0.5, 0, -5, true);
                break;
            case 3:
                chassis.DriveToPosition(0.5, 70, -5, true);
                break;
        }
        turnRobotToAngle(175);
        chassis.DriveToPosition(0.5, 0, -15, true);


    }



    public void CycleCone(boolean LR){
        if(LR){

        }
        /** First go to the stack of cones and grab a cone **/
        //Open the claw and swing the arm down
        claw.servo.setPosition(0);
        arm.swingDown();
        //Drive forward slightly
        chassis.DriveToPosition(0.6, 0, 25, true);
        //close the claw and grab onto the cone
        claw.servo.setPosition(1);
        /** Now drive to the medium pole **/
        //Drive to the pole and face it
        chassis.DriveToPosition(0.7,0,-60, true);
        turnRobotToAngle(210);
        chassis.stopDriveMotors();
        /** Now deliver the cone **/
        //Move the slider to the right height and swing down
       vSlider.MoveSlider(0.6, 1000,2400);
        arm.swingDown();
        //Open and close claw
        claw.servo.setPosition(1);
        // sleep(500); // TODO: Replace with while loop timer.
        claw.servo.setPosition(0);
        //swing arm back up
        arm.swingUp();
        //lower slider
       vSlider.MoveSlider(0.6, 0,1200);
        //Moves back to the stack
        turnRobotToAngle(90);
        chassis.stopDriveMotors();
        chassis.DriveToPosition(0.7,0,60, true);
    }

}