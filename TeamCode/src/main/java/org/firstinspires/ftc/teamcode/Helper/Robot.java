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

    // mechanisms.
    public DcMotor vSlider;
    public DcMotor swingArm;
    public Servo claw;

    private double holdingPower = -0.01;
    public double swingArmHoldingPower = 1;

    // global location.
    public int robotX = 0;
    public int robotY = 0;

    public int[] Location = {robotX,robotY};

    //IMU
    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;


    //Drivetrain Motor
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    //IMU
    //How many times the encoder counts a tick per revolution of the motor.
    static final double COUNTS_PER_MOTOR_REV_Hex= 538; // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    //Gear ratio of the motor to the wheel. 1:1 would mean that 1 turn of the motor is one turn of the wheel, 2:1 would mean two turns of the motor is one turn of the wheel, and so on.
    static final double DRIVE_GEAR_REDUCTION= 1; // This is < 1.0 if geared UP

    //Diameter of the wheel in CM
    static final double WHEEL_DIAMETER_CM= 10; // For figuring circumference

    //How many times the encoder counts a tick per CM moved. (Ticks per rev * Gear ration) / perimeter
    static final double COUNTS_PER_CM_Hex = (COUNTS_PER_MOTOR_REV_Hex * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_CM * 3.1415);


    // TFOD and Vuforia properties related to vision.
//    private static final String tfodModel = "jan2023mk2";
//    private static final String tfodPath = "/sdcard/FIRST/tflitemodels/" + tfodModel + ".tflite";
//    public static final String[] LABELS = {
//            "arrow",
//            "balloon",
//            "bar",
//            "pole",
//    };

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


    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        claw = hwMap.get(Servo.class, "claw");
        vSlider = hwMap.get(DcMotor.class, "vSlider");
        swingArm = hwMap.get(DcMotor.class, "swingArm");

        //Init motors and servos
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");



        //Setting the direction
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);


        claw.setDirection(Servo.Direction.FORWARD);
        vSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        swingArm.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set behavior when zero power is applied.
        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        swingArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Setting the run mode
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

         //Since our Rev Expansion is in Vertical Position, so we need to Z & X

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap y and z axes
//        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);
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

        robotY += distance;
        Location[1] = robotY;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = this.FLMotor.getCurrentPosition();
        int FRPos = this.FRMotor.getCurrentPosition();
        int BLPos = this.BLMotor.getCurrentPosition();
        int BRPos = this.BRMotor.getCurrentPosition();

        targetFR = FRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos + (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        this.FLMotor.setTargetPosition(targetFL);
        this.BLMotor.setTargetPosition(targetBL);
        this.FRMotor.setTargetPosition(targetFR);
        this.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.FLMotor.isBusy() && this.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void Strafe(double speed, int distance) {

        robotX += distance;

        runtime.reset();
        timeout_ms = 10000;

        robotX += distance;
        Location[0] = robotX;

        int targetFL;
        int targetFR;
        int targetBR;
        int targetBL;

        int FLPos = this.FLMotor.getCurrentPosition();
        int FRPos = this.FRMotor.getCurrentPosition();
        int BLPos = this.BLMotor.getCurrentPosition();
        int BRPos = this.BRMotor.getCurrentPosition();

        targetFR = FRPos - (int) (distance * COUNTS_PER_CM_Hex);
        targetBR = BRPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetFL = FLPos + (int) (distance * COUNTS_PER_CM_Hex);
        targetBL = BLPos - (int) (distance * COUNTS_PER_CM_Hex);

        //Set motor targets
        this.FLMotor.setTargetPosition(targetFL);
        this.BLMotor.setTargetPosition(targetBL);
        this.FRMotor.setTargetPosition(targetFR);
        this.BRMotor.setTargetPosition(targetBR);

        //set the mode to go to the target position
        this.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        FLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BLMotor.setPower(speed);
        BRMotor.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.FLMotor.isBusy() && this.FRMotor.isBusy())) {

        }
        this.stopDriveMotors();
    }

    public void stopDriveMotors(){
        // Stop all motion;
        this.FLMotor.setPower(0);
        this.FRMotor.setPower(0);
        this.BLMotor.setPower(0);
        this.BRMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        this.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        System.out.println(Arrays.toString(Location));
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
        angle = this.imu.getAngularOrientation();

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

        double pwr = -0.75;


        while (Math.abs(angleCurrent - angleEnd) > 10) {
            FLMotor.setPower(-pwr * direction);
            FRMotor.setPower(pwr * direction);
            BLMotor.setPower(-pwr * direction);
            BRMotor.setPower(pwr * direction);
            angle = this.imu.getAngularOrientation();
            angleCurrent = modAngle(angle.firstAngle);

        }
    }

    public void MoveSlider(double speed, int Position, int timeout) {
        timeout_ms = timeout;

        runtime.reset();

//        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlider.setTargetPosition(Position);
        //Set the power of the motor.
        vSlider.setPower(speed);
        //Run to position.
        vSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.milliseconds() < timeout_ms) && (vSlider.isBusy())) {
        }
        vSlider.setPower(0);
//        vSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        vSlider.setPower(0);
    }


    public void SwingArmToPosition(double speed, int Position) {
        timeout_ms = 3000;

        runtime.reset();

        this.swingArm.setTargetPosition(Position);

        //set the mode to go to the target position
        this.swingArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the power of the motor.
        swingArm.setPower(speed);

        while ((runtime.milliseconds() < timeout_ms) && (this.swingArm.isBusy())) {

        }
        this.swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Park(int location) {
        if (location == 1) {
            this.claw.setPosition(0);
            this.DriveToPosition(0.3, -75, 75, true);
        }

        if (location == 2) {
            this.claw.setPosition(0);
            this.DriveToPosition(0.3, 0, 75, true);
        }

        if (location == 3) {
            this.claw.setPosition(0);
            this.DriveToPosition(0.3, 75, 75, true);

        }
    }

    public void initArmClaw(){
        claw.setPosition(1);
        SwingArmToPosition(0.6,65);
        swingArm.setPower(swingArmHoldingPower);
        claw.setPosition(0);

    }

    public void deliverPreLoad(boolean LR) {
        /** First swing the arm up and go to the pole. **/
        //Close claw and swing the arm
        claw.setPosition(1);
        SwingArmToPosition(1, 65);
        swingArm.setPower(swingArmHoldingPower);
        //Drive to the pole
        if(LR) { // True = Left
            claw.setPosition(1);
            DriveToPosition(0.8, -8, 100, true);
            turnRobotToAngle(280);
        }
        else{
            claw.setPosition(1);
            DriveToPosition(0.8, 8, 100, true);
            turnRobotToAngle(90);
        }

        stopDriveMotors();
        /** Next, move the slider to the right height, swing the arm down, drop the cone, swing the arm back up, and lower the slider. **/
        //Moves the slider to the right height
        MoveSlider(1, 1000, 1850);
        //Swings the arm
        SwingArmToPosition(-1, 20);
        swingArm.setPower(0);
        //Opens and closes the claw to drop the cone
        claw.setPosition(0);
        timeout_ms = 500;
        runtime.reset();
        while ((runtime.milliseconds() < timeout_ms)) {

        }
        claw.setPosition(1);
        MoveSlider(1, 500, 300);
        //Swings the arm back up
        SwingArmToPosition(1, 65);
        swingArm.setPower(swingArmHoldingPower);
        while(swingArm.isBusy()) {

        }
        //lowers the slider
        MoveSlider(-1, 0, 1200);
    }

    public void ParkFromMedium(boolean LR, int location){
        if (LR) {
            turnRobotToAngle(350);
            stopDriveMotors();
        } else {
            turnRobotToAngle(160);
            stopDriveMotors();
        }

        MoveSlider(1, -500, 1200);

        if (location == 1) {
            claw.setPosition(0);
            DriveToPosition(0.3, 75, -30, true);
        }

        if (location == 2) {
            claw.setPosition(0);
            DriveToPosition(0.3, 0, -40, true);
        }

        if (location == 3) {
            claw.setPosition(0);
            DriveToPosition(0.3, -75, -40, true);

        }
    }



    public void CycleCone(boolean LR){
        if(LR){

        }
        /** First go to the stack of cones and grab a cone **/
        //Open the claw and swing the arm down
        claw.setPosition(0);
        SwingArmToPosition(1,20);
        //Drive forward slightly
        DriveToPosition(0.6, 0, 25, true);
        //close the claw and grab onto the cone
        claw.setPosition(1);
        /** Now drive to the medium pole **/
        //Drive to the pole and face it
        DriveToPosition(0.7,0,-60, true);
        turnRobotToAngle(210);
        stopDriveMotors();
        /** Now deliver the cone **/
        //Move the slider to the right height and swing down
        MoveSlider(0.6, 1000,2400);
        SwingArmToPosition(1, 20);
        //Open and close claw
        claw.setPosition(1);
        // sleep(500); // TODO: Replace with while loop timer.
        claw.setPosition(0);
        //swing arm back up
        SwingArmToPosition(1, 65);
        swingArm.setPower(swingArmHoldingPower);
        //lower slider
        MoveSlider(0.6, 0,1200);
        //Moves back to the stack
        turnRobotToAngle(90);
        stopDriveMotors();
        DriveToPosition(0.7,0,60, true);
    }

}