package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class MotorControl {
    public DcMotor rotater  = null;
    HardwareMap hwMap = null;
    public DcMotor intakeOne  = null;
    public DcMotor VertLift = null;
    public DcMotor HorzLift = null;
    public MotorControl() {
        //Constructor
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motor
        rotater = hwMap.get(DcMotor.class, "rotater");
        intakeOne = hwMap.get(DcMotor.class, "intakeOne");
        VertLift = hwMap.get(DcMotor.class, "VertLift");
        HorzLift = hwMap.get(DcMotor.class, "HorzLift");

        //define motor direction
        rotater.setDirection(DcMotor.Direction.REVERSE);
        intakeOne.setDirection(DcMotor.Direction.REVERSE);
        VertLift.setDirection(DcMotor.Direction.FORWARD);
        HorzLift.setDirection(DcMotor.Direction.FORWARD);


        rotater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VertLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HorzLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        rotater.setPower(0);
        intakeOne.setPower(0);
        VertLift.setPower(0);
        HorzLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VertLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HorzLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void rotaterPower(double power) {
        rotater.setPower(-power);
    }
    public double rotaterPower() {
        return rotater.getPower();
    }
    public void intakeOnePower(double power) {

        intakeOne.setPower(-power);
    }
    public double intakeOnePower() {

        return intakeOne.getPower();
    }
}
