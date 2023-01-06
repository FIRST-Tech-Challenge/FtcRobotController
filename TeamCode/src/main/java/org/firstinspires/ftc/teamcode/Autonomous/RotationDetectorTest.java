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

    // Declare motors
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    // Initialize collector servo and CRServo
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

        // Initialize motors
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        // Create instance of Collector class using the initialized CRServo
        collector = new Collector(collectorCr);

        // Create instances of Move and Rotate classes using the initialized motors
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);

        // Get the first VoltageSensor in the hardware map and create an instance of the
        // VoltageReader class using it
        VoltageSensor VS = this.hardwareMap.voltageSensor.iterator().next();
        voltageReader = new VoltageReader(VS);

        // Get the BNO055IMU in the hardware map and create an instance of the RotationDetector
        // class using it
        rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMU.class, "imu"));

        // Wait for the start button to be pressed
        waitForStart();



        // Test rotation to 30 degrees
        double angle = 30.0;
        while(opModeIsActive() && rotationDetector.WaitForRotation(angle))
        {
            //try {
                // Rotate the robot using the RotateRaw method and the MotorPower method from the
                // RotationDetector class
                rotate.RotateRaw(2, rotationDetector.MotorPower(angle));

            // Display debugging data on the telemetry
                DebugData(angle);
            //}
            //catch (Exception e) {
            //    telemetry.addData("Exception caught in MotorPower(RotationDetectorTest): ", e.getMessage());
            //    telemetry.addData("MotorPower angle is: ", angle);
            //    telemetry.update();
            //}
        }

        // Stop the robot
        rotate.MoveStop();

        // Delay for 2 seconds
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

    /**
     * Display debugging data on the telemetry.
     *
     * @param angle The target angle for the robot to rotate to.
     * //@param telemetryOn Whether to display the data on the telemetry.
     */
    void DebugData(double angle){

        // Get the current orientation of the robot
        double[] orientation = rotationDetector.getOrientation();

        // Build a string representation of the orientation values
        StringBuilder orientationString = new StringBuilder();
        orientationString.append("Orientation (Z: ").append(orientation[0]);
        orientationString.append(", Y: ").append(orientation[1]);
        orientationString.append(", X: ").append(orientation[2]).append(")");

        // Add data to the telemetry
        telemetry.addData("PID Parameters: ", rotationDetector.getPIDParameters());
        telemetry.addData("Current rotation", rotationDetector.ReturnPositiveRotation());
        telemetry.addData(" - ", orientationString.toString());
        telemetry.addData("Target rotation", angle);
        telemetry.update();
    }

}

