package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;

//Todo
// Servo and Linkage are non-linear so servo range is 0 degrees -> 90 degrees with .2 being 0 and 1 being 90 degrees
// This makes height & angle not accurate at lower values but it seems good enough and i suspect usage this won't matter
// as I think wrist should go to a standard position in space
// but the todo is linkage math to make the telemetry work better

public class Wrist {

    //Wrist Constants
    private static final double DEG_TO_SERVO_POS = 180;
    private static final double WRIST_RADIUS = 7.338;
    private static final double WRIST_MAX_HEIGHT = 7.5;
    private static final double WRIST_MIN_HEIGHT = 4;
    private static final double WRIST_MAX_POSITION = 1;
    private static final double WRIST_MIN_POSITION = .2;
    private static final double WRIST_PHYSICAL_SEARCH_STEP = .001;
    //wrist angle constants
    private static final double a = 64; //Unit: mm
    private static final double b = 104;//Unit: mm
    private static final double c = 106.73;//Unit: mm
    private static final double d = 68.8186;//Unit: mm
    private double A,B,C;

    //wrist private variables
    private ServoImplEx wrist_servo;
     private boolean wrist_is_busy = false;
    private double targetPosition = 0;
    private double currentWristServoAngle;
    private double currentServoPosition;
    private double targetHeight = 0;
    private static ElapsedTime wristStopwatch = new ElapsedTime();
    private double currentAngle;
    private double calcPosition = -1;

    public Wrist(HardwareMap hm) {
        init(hm);
    }
    private void init(HardwareMap hm) {
        wrist_servo = hm.get(ServoImplEx.class, "wrist");
    }

    public void GoToHeight(double target_height) {
        double temp;
        if(target_height > WRIST_MAX_HEIGHT){target_height = WRIST_MAX_HEIGHT;}
        if(target_height < WRIST_MIN_HEIGHT){target_height = WRIST_MIN_HEIGHT;}
        targetHeight = target_height;
        temp = target_height / WRIST_RADIUS;
        temp = Math.asin(temp);
        temp = Math.toDegrees(temp);
        targetPosition = temp/DEG_TO_SERVO_POS;
        this.SetPos(temp/DEG_TO_SERVO_POS);
        wrist_is_busy = true;
        wristStopwatch.reset();
        //wrist_servo.setPwmRange(PwmControl.PwmRange.defaultRange);
    }
    public void GoToAngle(double target_angle) {

        this.targetPosition =  target_angle/DEG_TO_SERVO_POS;
        this.SetPos(targetPosition);
        wrist_is_busy = true;
        wristStopwatch.reset();
    }

    public void GoToPhysicalAngle(double target_angle) {
        // GetAngle() is linkage angle, while GoToAngle() is a servo-command angle.
        // This helper searches the servo range for the raw position that best produces
        // the requested physical linkage angle used by IK and telemetry.
        this.targetPosition = servoPositionForPhysicalAngle(target_angle);
        this.SetPos(targetPosition);
        wrist_is_busy = true;
        wristStopwatch.reset();
    }
    public double GetAngle() {
        A = (2*c*d)-(2*a*c*(Math.cos(Math.toRadians(260.5377-currentWristServoAngle)))); //=(2*$C$3*$D$3)-(2*$A$3*$C$3*(COS(RADIANS(260.5377-$B18))))
        B = -(2*a*c*(Math.sin(Math.toRadians(260.5377-currentWristServoAngle)))); //=-(2*$A$3*$C$3*(SIN(RADIANS(260.5377-B18))))
        C = Math.pow(a,2)-Math.pow(b,2)+Math.pow(c,2)+Math.pow(d,2)-(2*d*a*(Math.cos(Math.toRadians(currentWristServoAngle-260.5377))));
        //=POWER($A$3,2)-POWER($B$3,2)+POWER($C$3,2)+POWER($D$3,2)-(2*$D$3*$A$3*(COS(RADIANS(B18-260.5377))))
        //A = (2*c*d)-(2*a*c*(Math.toDegrees(Math.cos(260.5377-Math.toRadians(currentWristServoAngle)))));
       // B = -(2*a*c*(Math.toDegrees(Math.sin(260.5377-Math.toRadians(currentWristServoAngle)))));
       // C = Math.pow(a,2)-Math.pow(b,2)+Math.pow(c,2)+Math.pow(d,2)-(2*d*a*(Math.toDegrees(Math.cos(Math.toRadians(currentWristServoAngle)-260.5377))));
        currentAngle = 226.9955 - (2*( Math.toDegrees(Math.atan(((-1*B)-Math.sqrt((Math.pow(B,2)-Math.pow(C,2)+Math.pow(A,2))))/(C-A)))));
        //=226.9955-(2*(DEGREES(ATAN(((-1*B7)-SQRT((POWER(B7,2))-(POWER(B8,2))+(POWER(B6,2))))/(B8-B6)))))
        return currentAngle;
    }
    public double GetServoAngle() {
        currentWristServoAngle = currentServoPosition * DEG_TO_SERVO_POS; //DEG_TO_SERVO_POS = 180
        currentWristServoAngle = (180-currentWristServoAngle)+73.8; //.410 = either -180 or 180 degrees
        return currentWristServoAngle;
    }
    public double GetHeight() {
        double temp;
        temp = currentAngle;
        temp = Math.toRadians(temp);
        temp = Math.sin(temp);
        return (WRIST_RADIUS *temp);
    }
    public double GetLength(){
        double temp;
        temp = currentAngle;
        temp = Math.toRadians(temp);
        temp = Math.cos(temp);
        return (WRIST_RADIUS *temp);
    }
    public double GetRawPos() {return currentServoPosition;}
    public double GetTargetPos() {return targetPosition * DEG_TO_SERVO_POS;}
    public double GetRawTargetPos() {return targetPosition;}
    public double GetTargetHeight() {return targetHeight;}
    public double GetA() {return A;}
    public double GetB() {return B;}
    public double GetC() {return C;}
    public double GetCalcPos(){return calcPosition;}
    public boolean IsBusy() {return wrist_is_busy;}

    public void SetPos(double power) {
        wrist_is_busy = false;
        //safety checks
        power = Math.min(Math.max(power, WRIST_MIN_POSITION), WRIST_MAX_POSITION);
        wrist_servo.setPosition(power);
    }
    public void SetTargetPos(double power) {
        wrist_is_busy = true;
        wristStopwatch.reset();
        //safety checks
        power = Math.min(Math.max(power, WRIST_MIN_POSITION), WRIST_MAX_POSITION);
        wrist_servo.setPosition(power);
    }
    public void SetLevel(double shoulder_angle){
        // Shoulder-compensation calibration: returns the wrist command that keeps the nozzle level.
        calcPosition = levelAngleForShoulder(shoulder_angle);
        GoToPhysicalAngle(calcPosition);
    }

    public static double levelAngleForShoulder(double shoulder_angle) {
        // Leveling rule: start from horizontal wrist, then offset by shoulder angle
        // so the wrist rotates with the shoulder and stays parallel to the ground.
        double levelAngle = RobotGeometry.TEST_IK_LEVEL_WRIST_POSE_ANGLE + shoulder_angle;
        return RobotGeometry.clamp(
                levelAngle,
                RobotGeometry.TEST_IK_WRIST_MIN_ANGLE,
                RobotGeometry.TEST_IK_WRIST_MAX_ANGLE);
    }

    public static double servoPositionForPhysicalAngle(double targetAngle) {
        double bestPosition = WRIST_MIN_POSITION;
        double bestError = Double.MAX_VALUE;

        for (double position = WRIST_MIN_POSITION; position <= WRIST_MAX_POSITION; position += WRIST_PHYSICAL_SEARCH_STEP) {
            double angle = physicalAngleForServoPosition(position);
            double error = Math.abs(angle - targetAngle);
            if (error < bestError) {
                bestError = error;
                bestPosition = position;
            }
        }

        return bestPosition;
    }

    private static double physicalAngleForServoPosition(double servoPosition) {
        double wristServoAngle = (180 - (servoPosition * DEG_TO_SERVO_POS)) + 73.8;
        double localA = (2*c*d)-(2*a*c*(Math.cos(Math.toRadians(260.5377-wristServoAngle))));
        double localB = -(2*a*c*(Math.sin(Math.toRadians(260.5377-wristServoAngle))));
        double localC = Math.pow(a,2)-Math.pow(b,2)+Math.pow(c,2)+Math.pow(d,2)
                -(2*d*a*(Math.cos(Math.toRadians(wristServoAngle-260.5377))));

        double discriminant = Math.pow(localB, 2) - Math.pow(localC, 2) + Math.pow(localA, 2);
        if (discriminant < 0) {
            return Double.NaN;
        }

        return 226.9955 - (2*(Math.toDegrees(Math.atan(((-1*localB)-Math.sqrt(discriminant))/(localC-localA)))));
    }
    public void Stop() {
        wrist_is_busy = false;
    }

    public void Update() {
        currentServoPosition = wrist_servo.getPosition();
        currentWristServoAngle = GetServoAngle();
        currentAngle = GetAngle();
        if (wrist_is_busy && wristStopwatch.milliseconds() > 300) {wrist_is_busy = false;}
    }
}

