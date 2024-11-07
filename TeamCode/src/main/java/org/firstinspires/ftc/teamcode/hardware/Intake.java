package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    private Servo arm = null;
    private Servo armWrist = null;
    private Servo wrist = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    public void Init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(Servo.class, "arm");
        armWrist = hardwareMap.get(Servo.class, "armwrist");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
    }
    public void ArmPickup() {
        // make arm ready to pickup sample/specimen
        arm.setPosition(1.0);
    }
    public void  ArmTransfer() {
        // make arm ready to give sample/specimen to main claw
        arm.setPosition(-1.0);
    }
    public void ArmWristPickup() {
        // make arm ready to pickup sample/specimen
        armWrist.setPosition(-1.0);
    }
    public void  ArmWristDrive() {
        // bring arm wrist up so we can drive without issues
        armWrist.setPosition(0.0);
    }
    public void  ArmWristTransfer() {
        // bring arm wrist up to main claw to transfer
        armWrist.setPosition(1.0);
    }
    public void WristCenter() {
        // center the wrist(whatever center is)
        wrist.setPosition(0.0);
    }
    public void WristRotateLeft() {
        double currentPos = wrist.getPosition();
        wrist.setPosition(currentPos -= 0.01);
    }
    public void WristRotateRight(){
        double currentPos = wrist.getPosition();
        wrist.setPosition(currentPos += 0.01);
    }
    public void ClawGrab() {
        // make claw make spinny things rotate so block go in
        claw1.setDirection(Servo.Direction.FORWARD); // swap directions if nesscary
        claw2.setDirection(Servo.Direction.FORWARD);
    }
    public void ClawRelease() {
        // make claw make spinny things rotate so block go out
        claw1.setDirection(Servo.Direction.REVERSE);
        claw2.setDirection(Servo.Direction.REVERSE); // swap directions if nesscary
    }
    public void  ClawStop() {
        // make claw stop spinny things so it dont make block go in or out
        claw1.setPosition(claw1.getPosition());
        claw2.setPosition(claw2.getPosition());
    }
}
