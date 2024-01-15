package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

public class  RobotHardware implements Runnable{
    /**
     *  to use RobotHardware use this:
     *
     *      RobotHardware H = new RobotHardware();
     *
     *  then run this in runOpMode()
     *
     *      H.init(hardwareMap)
     *
     *  if you want to read a sensor value or change motor speed use this:
     *
     *      H.[sensor/motor name].[function name]([var 1], [var 2] ...);
     */
    
    public HardwareMap hardwareMap;
    LinearOpMode opMode;
    RobotTracker tracker;

    ////////////////////////////// Constants //////////////////////////////

    public final DcMotor.Direction[] MOTOR_DIRECTION = {DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD};

    public final double GRABBER_SERVO_OPEN = 0.67;
    public final double GRABBER_SERVO_CLOSED = 0.5;
    
    private final double COUNTS_PER_REVOLUTION = 560;
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    public final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);
    public final double MINIMUM_MOTOR_POWER = 0.1;
    
    public final double EXP_BASE = 20;
    public final double INITIAL_VALUE = 0.05;
    public final double STICK_DEAD_ZONE = 0.05;
    public final double POWER_FOLLOW_INCREMENT = 0.05;
    public final double MAX_ACCELERATION_TALL = 6;
    public final double MAX_ACCELERATION_SHORT = 17;
    public final double MAX_LIFT_POS = 2100;
    public final double MAX_VELOCITY = 2.5;
    public final double ROOT_TWO = Math.sqrt(2);
    
    public final int[] nerd = {1,1,1,0,1,0,0,0,1,0,0,0,1,0,1,1,1,0,1,0,0,0,1,1,1,0,1,0,1,0,0,0,0,0,0,0};
    
    final int[] MB1242_SENSOR_OFFSET = {6,-1,6,1};
    
    final long TARGET_LOOP_DURATION = 100;
    
    public final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    
    public static double headingSave = 0;
    private long loopStart;
    
    ////////////////////////////// Toggles //////////////////////////////
    
    boolean XYEncoderEnable = false;
    boolean cameraEnable = false;
    boolean LEDEnable = false;
    
    ////////////////////////////// Output variables //////////////////////////////
    
    public double heading = 0;
    public double rightDistance = 0;
    public double frontDistance = 0;
    public double[] range = {0,0,0,0};
    public long liftPos = 0;
    public long liftZero = 0;
    public long climbZero = 0;


    ////////////////////////////// Sensors //////////////////////////////

    //public DistanceSensor frontRange;
    //public Rev2mDistanceSensor frontTOF;
    public static BNO055IMU      imu;
    public static BNO055IMU      imu1;
    public Orientation    angles;
    public Orientation    angles1;
    //public WebcamName webcam;
    //public int monitorViewId;
    public DcMotor yEncoder;
     public DcMotor xEncoder;
    public ElapsedTime runtime = new ElapsedTime();
    //public DigitalChannel liftStop;
    //public MB1242Ultrasonic[] MB1242 = new MB1242Ultrasonic[4];
    public boolean newRangeDataFlag = false;
    
    ////////////////////////////// LEDs //////////////////////////////
    public QwiicLED LED;
    int LEDMode = 2;
    int LEDCycle = 0;
    int LiftLEDs = 0;
    int TeamColor = Color.RED;

    ////////////////////////////// Motors //////////////////////////////
    
    //public DcMotor        collectorMotor;
    public DcMotor        liftMotor;
    public DcMotor        climbMotor;
    public DcMotorEx[]      driveMotor = new DcMotorEx[4];
    //public Servo[]        wheelLift = new Servo[4];
    //public Servo          duckServo;
    //public Servo          rampServo;
    //public Servo          collectorServo;
    //public Servo          clawServo;
    //public Servo          distancePlatform;

    public void init(HardwareMap HM, LinearOpMode telOp) {
        
        runtime.reset();
        
        hardwareMap = HM;
        this.opMode = telOp;

        TeamColor = Color.rgb(255*(1-autonomous.pathSide),0,255*autonomous.pathSide);
        
        ////////////////////////////// Hardware Map //////////////////////////////

        driveMotor[0] = HM.get(DcMotorEx.class, "3");
        driveMotor[1] = HM.get(DcMotorEx.class, "0");
        driveMotor[2] = HM.get(DcMotorEx.class, "2");
        driveMotor[3] = HM.get(DcMotorEx.class, "1");
        liftMotor     = HM.get(DcMotor.class, "Lift_Motor");
        climbMotor    = HM.get(DcMotor.class, "Climb_Motor");
        
        //clawServo = HM.get(Servo.class, "C_Servo");
    /*
        if (cameraEnable) {
            webcam = HM.get(WebcamName.class, "Webcam 1");
            monitorViewId = HM.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", HM.appContext.getPackageName());
        }*/
        
//        MB1242[0] = HM.get(MB1242Ultrasonic.class, "MB0");
//        MB1242[1] = HM.get(MB1242Ultrasonic.class, "MB1");
//        MB1242[2] = HM.get(MB1242Ultrasonic.class, "MB2");
//        MB1242[3] = HM.get(MB1242Ultrasonic.class, "MB3");
//        frontRange = HM.get(DistanceSensor.class, "F_Range");
        imu = HM.get(BNO055IMU.class, "imu");
        //imu1 = HM.get(BNO055IMU.class, "imu1");
    /*
        if (XYEncoderEnable) {
            yEncoder = HM.get(DcMotor.class, "Y_Motor");
            xEncoder = HM.get(DcMotor.class, "X_Motor");
    
            yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            yEncoder.setDirection(DcMotor.Direction.FORWARD);
            xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            xEncoder.setDirection(DcMotor.Direction.REVERSE);
            
            tracker = new RobotTracker(this);
        }
    */
 //       liftStop = HM.get(DigitalChannel.class, "Lift_Digital");
        
 //       LED = HM.get(QwiicLED.class, "LED");
 //       LED.changeWriteDelay(200);
 //       LED.changeLength(70);
 //       LED.setBrightness(2);

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);
        //imu1.initialize(parameters);
        
//        frontTOF = (Rev2mDistanceSensor) frontRange;

        for (int i = 0; i < 4; i++) {
            driveMotor[i].setDirection(MOTOR_DIRECTION[i]);
            driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbMotor.setDirection(DcMotor.Direction.FORWARD);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    
    /*void updateLEDs() {
        switch (LEDMode) {
            case 0: // rainbow
                LED.setWalkingColor(Color.HSVToColor(new float[]{LEDCycle * 5, 1, 1}), 2, 10, 30, false);
                LED.setWalkingColor(Color.HSVToColor(new float[]{LEDCycle * 5, 1, 1}), 2, 40, 30, true);
                break;
            case 1: // nerd morse code
                LED.setWalkingColor(Color.red(nerd[LEDCycle % 36]), 2, 10, 60, false);
                break;
            case 2: // Lift indicator
                int NumLEDs = Range.clip(Range.clip((int)(liftPos/MAX_LIFT_POS * 30),0,30) - LiftLEDs, -12, 12);
                if (NumLEDs == 0) return;
                if (NumLEDs > 0) {
                    LED.setLEDColorSegment(TeamColor, (40 - LiftLEDs - NumLEDs), NumLEDs);
                    LED.setLEDColorSegment(TeamColor, (40 + LiftLEDs), NumLEDs);
                } else {
                    LED.setLEDColorSegment(Color.BLACK, (40 - LiftLEDs), -NumLEDs);
                    LED.setLEDColorSegment(Color.BLACK, (40 + LiftLEDs + NumLEDs), -NumLEDs);
                }
                LiftLEDs += NumLEDs;
                break;
            case 3: // Cone detection
                LED.turnAllOff();
                switch (autonomous.color) {
//                    case 0:
                        LED.setLEDColorSegment(Color.MAGENTA, 31, 6);
                        break;
                    case 1:
                        LED.setLEDColorSegment(Color.CYAN, 37, 6);
                        break;
                    case 2:
                        LED.setLEDColorSegment(Color.YELLOW, 43, 6);
                        break;
                }
                break;
            case 4:
                LED.turnAllOff();
        }
        
        LEDCycle += 1;
        if (LEDCycle >  72) {
            LEDCycle = 0;
        }
    }*/
    
    public void run() {
        
        while (!isStopRequested()) {
            
            loopStart = runtime.time(TimeUnit.MILLISECONDS);
    
            //for (MB1242Ultrasonic mb1242Ultrasonic : MB1242) mb1242Ultrasonic.range();
    
            heading = getheading();
            /*
            frontDistance = frontTOF.getDistance(DistanceUnit.INCH);
    
            if (!liftStop.getState()) {
                liftZero = liftMotor.getCurrentPosition();
            }
            liftPos = liftMotor.getCurrentPosition() - liftZero;
            
            if (LEDEnable) {
                updateLEDs();
            } else {
                LED.turnAllOff();
            }

            if (XYEncoderEnable) {
                tracker.Iterate();
            }
            
            if (runtime.time(TimeUnit.MILLISECONDS) - loopStart < TARGET_LOOP_DURATION-5) {
                //Log.d("SensLoop","" + (runtime.time(TimeUnit.MILLISECONDS)-loopStart));
                try {
                    Thread.sleep(TARGET_LOOP_DURATION - 5 - (runtime.time(TimeUnit.MILLISECONDS)-loopStart));
                } catch (Exception e) {
                    Log.e("SensLoop", e.getMessage());
                }
                //while ((loopStart + TARGET_LOOP_DURATION-5) > runtime.time(TimeUnit.MILLISECONDS)) {
                //    opMode.idle();
                //}
            } else {
                Log.w("SensLoop", "loop duration exceeds target time frame: " + (runtime.time(TimeUnit.MILLISECONDS)-loopStart));
            }
            /*for (int i = 0; i < MB1242.length; i++) {
                range[i] = MB1242[i].inches() + MB1242_SENSOR_OFFSET[i];
                newRangeDataFlag = true;
            }*/
            try {
                Thread.sleep(5);
            } catch (Exception e) {
                Log.e("SensLoop", e.getMessage());
            }
            //for (int i = 3; i >= 0; i--) driveEncoder[0] = driveMotor[0].getCurrentPosition();
            //armVoltage = armAngle.getVoltage();
    
        }
        
    }
    

    public void setLiftPos(int pos) {
        liftMotor.setTargetPosition((int)(pos + liftZero));
        liftMotor.setPower(0.7);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setClimbPos(int pos) {
        climbMotor.setTargetPosition((int)(pos + climbZero));
        climbMotor.setPower(0.7);
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void setGrabber(boolean open) {
        /*
        if (open) {
            clawServo.setPosition(GRABBER_SERVO_OPEN);
        } else {
            clawServo.setPosition(GRABBER_SERVO_CLOSED);
        }
         */
    }


    public double getheading() {
        // returns a value between -180 and 180, forwards is 0 when initialised
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //return (angles.firstAngle + angles1.firstAngle)/2;
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + headingSave);// + AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))/2;
    }
    
    public void saveHeading() {
        headingSave = getheading();
    }
    
    public boolean isStopRequested() {
        return opMode.isStopRequested();
    }
    
    public void setCameraEnable(boolean cameraEnable) {
        
        this.cameraEnable = cameraEnable;
    }
    
    public void setXYEncoderEnable(boolean XYEncoderEnable) {
        
        this.XYEncoderEnable = XYEncoderEnable;
    }
    
}
