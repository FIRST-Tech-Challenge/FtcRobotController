package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.gary;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="practice", group="Linear Opmode")
public class Mecanum_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    double finalAngle;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double speed = 0.5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.right_bumper == true) {
                while (gamepad1.right_bumper) {
                }
                speed += 0.05;
            }

            if (gamepad1.left_bumper == true) {
                while (gamepad1.left_bumper) {
                }
                speed -= 0.05;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double lF_P;
            double lB_P;
            double rF_P;
            double rB_P;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double dRL = -gamepad1.left_stick_y;
            double drive = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            lF_P = Range.clip(speed * (drive + turn - dRL), -1.0, 1.0);
            rF_P = Range.clip(speed * (drive + turn + dRL), -1.0, 1.0);
            lB_P = Range.clip(speed * (drive - turn + dRL), -1.0, 1.0);
            rB_P = Range.clip(speed * (drive - turn - dRL), -1.0, 1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftFront.setPower(lF_P);
            rightFront.setPower(rF_P);
            leftBack.setPower(lB_P);
            rightBack.setPower(rB_P);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            double heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);


            if(heading >= 0){
                finalAngle = heading;
            }
            else{
                finalAngle = heading+360;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", lF_P, rF_P);
            telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", lB_P, rB_P);
            telemetry.addData("Speed:", speed);
            telemetry.addData("heading", "%.1f", finalAngle);
            telemetry.update();


        }
    }

    /*public void Sensor() {
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        double heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.addData("heading", "%.1f", heading);
            telemetry.update();

        }





    }*/




}

