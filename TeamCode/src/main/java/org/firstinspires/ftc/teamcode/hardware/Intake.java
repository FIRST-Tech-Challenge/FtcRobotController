package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    private Servo armServo1 = null;
    private Servo armWristServo = null;
    private Servo wristServo = null;
    private Servo clawServo1 = null;
    private Servo clawServo2 = null;
    public void Init(HardwareMap hardwareMap) {
//        armServo1 = hardwareMap.get(Servo.class, "arm1");
//        armServo2 = hardwareMap.get(Servo.class, "arm2");
//        armWristServo = hardwareMap.get(Servo.class, "armwristservo");
        wristServo = hardwareMap.get(Servo.class, "wrist");
//        clawServo1 = hardwareMap.get(Servo.class, "claw1");
//        clawServo2 = hardwareMap.get(Servo.class, "claw2");
    }
    public void ArmPickup() {
        // make arm ready to pickup sample/specimen
        armServo1.setPosition(1.0);
    }
    public void  ArmTransfer() {
        // make arm ready to give sample/specimen to main claw
        armServo1.setPosition(-1.0);
    }
    public void ArmWristPickup() {
        // make arm ready to pickup sample/specimen
        armWristServo.setPosition(-1.0);
    }
    public void  ArmWristDrive() {
        // bring arm wrist up so we can drive without issues
        armWristServo.setPosition(0.0);
    }
    public void  ArmWristTransfer() {
        // bring arm wrist up to main claw to transfer
        armWristServo.setPosition(1.0);
    }
    public void WristCenter() {
        // center the wrist(whatever center is)
        wristServo.setPosition(0.0);
    }
    public void WristRotateLeft() {
        double currentPos = wristServo.getPosition();
        wristServo.setPosition(currentPos -= 0.01);
    }
    public void WristRotateRight(){
        double currentPos = wristServo.getPosition();
        wristServo.setPosition(currentPos += 0.01);
    }
    public void ClawGrab() {
        // make claw make spinny things rotate so block go in
        clawServo1.setDirection(Servo.Direction.FORWARD); // swap directions if nesscary
        clawServo1.setDirection(Servo.Direction.FORWARD);
    }
    public void ClawRelease() {
        // make claw make spinny things rotate so block go out
        clawServo1.setDirection(Servo.Direction.REVERSE);
        clawServo1.setDirection(Servo.Direction.REVERSE); // swap directions if nesscary
    }
    public void  ClawStop() {
        // make claw stop spinny things so it dont make block go in or out
        clawServo1.setPosition(clawServo1.getPosition());
        clawServo2.setPosition(clawServo2.getPosition());
    }
}
