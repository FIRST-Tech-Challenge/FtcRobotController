package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@Autonomous(name = "RotationDetectorTest", group = "Concept")
@Disabled
public class RotationDetectorTest extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack, armMotorLeft, armMotorRight;
    private Servo collectorServo;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Collector collector;
    public VoltageReader voltageReader;
  //  public NewCorrectionRotateMove2 AutoCorrection;
    //public MoveAutocorrectTest MoveAutocorrectTest;
    public RotationDetector rotationDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        //collectorServo = hardwareMap.servo.get("CL");
        //collectorCr = hardwareMap.crservo.get("CS");
        collector = new Collector(collectorCr);
        //arm = new Arm(armMotorLeft, armMotorRight);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        //AutoCorrection = new NewCorrectionRotateMove2(rotationDetector,move,rotate);
        //MoveAutocorrectTest = new MoveAutocorrectTest(rotationDetector, move);
        waitForStart();

        int angle = 180;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
            DebugData(angle);
        }
        rotate.MoveStop();
        sleep(2000);
        angle = 0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
            DebugData(angle);

        }
        rotate.MoveStop();
        sleep(2000);
        angle = 160;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
            DebugData(angle);

        }
        rotate.MoveStop();
        sleep(1000);
        angle = 45;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
            DebugData(angle);
        }
        rotate.MoveStop();
    }

    void DebugData(float angle){
        telemetry.addData("Current rotation", rotationDetector.ReturnPositiveRotation());
        telemetry.addData("Target rotation", angle);
        telemetry.update();
    }

}

