package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.ArmServos;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.Vacuum;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@Autonomous(name="AutonomieShowOff", group="TEST")
public class AutonomieShowOff extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack,vaccumLeft, armMotorChain;
    private DcMotorEx armMotorLeft, armMotorRight;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Vacuum vaccum;
    private RotationDetector rotationDetector;
    private Collector collector;
    public VoltageReader voltageReader;
    private Servo L1Servo;
    private Servo L2Servo;
    public MoveAutocorrect2 AutoCorrection;
    private ArmServos armServos;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        vaccumLeft = hardwareMap.dcMotor.get("VL");
        L1Servo = hardwareMap.servo.get("L1S");
        L2Servo = hardwareMap.servo.get("L2S");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
        armMotorChain = hardwareMap.get(DcMotor.class, "AMC");
        arm = new Arm(armMotorLeft, armMotorRight, armMotorChain);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        vaccum = new Vacuum(vaccumLeft);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        collectorCr = hardwareMap.crservo.get("CR");
        collector = new Collector(collectorCr);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        armServos = new ArmServos(L1Servo, L2Servo);
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);

        waitForStart();

        move.MoveFull(2);
        sleep(voltageReader.GetWaitTime(20, 1));
        move.MoveStop();

        arm.Start(-1);
        sleep(2000);
        armServos.Level1Up();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        int angle = 360;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        angle = 45;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        angle = 45;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(1,rotationDetector.MotorPower(angle));
        }
        rotate.MoveStop();
        sleep(500);

        arm.Start(-1);
        sleep(1000);
        armServos.Level1Down();
        sleep(1000);
        arm.Stop();
        sleep(1000);

        move.MoveFull(1);
        sleep(voltageReader.GetWaitTime(20, 1));
        move.MoveStop();

    }
}
