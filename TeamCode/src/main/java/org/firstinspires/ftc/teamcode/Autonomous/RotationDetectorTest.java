package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Functions.Arm;
import org.firstinspires.ftc.teamcode.Functions.Collector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;

@Autonomous(name = "RotationDetectorTest", group = "Concept")
public class RotationDetectorTest extends LinearOpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private Servo collectorServo;
    private CRServo collectorCr;
    private Move move;
    private Rotate rotate;
    private Arm arm;
    private Collector collector;
    public VoltageReader voltageReader;
    public RotationDetector rotationDetector;
    //public Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        collector = new Collector(collectorCr);
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));
        waitForStart();

        double angle = 30.0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            //try {
                rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
                DebugData(angle);
            //}
            //catch (Exception e) {
            //    telemetry.addData("Exception caught in MotorPower(RotationDetectorTest): ", e.getMessage());
            //    telemetry.addData("MotorPower angle is: ", angle);
            //    telemetry.update();
            //}
        }

        rotate.MoveStop();
        sleep(2000);



        angle = -45.0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
            DebugData(angle);

        }
        rotate.MoveStop();
        sleep(2000);



        angle = 90.0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2, rotationDetector.MotorPower(angle));
            DebugData(angle);

        }
        rotate.MoveStop();
        sleep(1000);



        angle = 45.0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            rotate.RotateRaw(2,rotationDetector.MotorPower(angle));
            DebugData(angle);
        }
        rotate.MoveStop();
    }

    void DebugData(double angle){
        double[] orientation = rotationDetector.getOrientation();

        StringBuilder orientationString = new StringBuilder();
        orientationString.append("Orientation (Z: ").append(orientation[0]);
        orientationString.append(", Y: ").append(orientation[1]);
        orientationString.append(", X: ").append(orientation[2]).append(")");

        telemetry.addData("PID Parameters: ", rotationDetector.getPIDParameters());
        telemetry.addData("Current rotation", rotationDetector.ReturnPositiveRotation());
        telemetry.addData(" - ", orientationString.toString());
        telemetry.addData("Target rotation", angle);
        telemetry.update();
    }

}

