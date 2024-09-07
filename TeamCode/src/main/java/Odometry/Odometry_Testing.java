package Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Odometry_Testing extends LinearOpMode {

    // Declare OpMode members for each of the 3 motors and IMU.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor odom_l, odom_r, odom_h;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        odom_l = hardwareMap.get(DcMotor.class, "odom_l");
        odom_r = hardwareMap.get(DcMotor.class, "odom_r");
        odom_h = hardwareMap.get(DcMotor.class, "odom_h");

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.addData("Status", "Program Initialized");
        telemetry.update();

        odom_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom_h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odom_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odom_h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double prev_encoder_l = 0, prev_encoder_r = 0, prev_encoder_h = 0, prev_ang = 0, current_ang ;
        double delta_encoder_l, delta_encoder_r, delta_encoder_h, delta_local_x, delta_local_y, delta_global_x, delta_global_y, delta_ang;
        double global_xM = 0, global_yM = 0;
        double disM_encoderHtoCenter = 0.235; // Distance from the horizontal encoder to the center of the robot in meters

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // Convert encoder ticks to meters
            double encoder_l = encoderToMetres(odom_l.getCurrentPosition());
            double encoder_r = encoderToMetres(odom_r.getCurrentPosition());
            double encoder_h = encoderToMetres(-odom_h.getCurrentPosition());
            telemetry.addData("l", encoder_l);
            telemetry.addData("r", encoder_r);
            telemetry.addData("h", encoder_h);

            // Get current angle from IMU
            current_ang = Math.toRadians(getAngle());

            // Calculate changes in encoder values and angle
            delta_encoder_l = encoder_l - prev_encoder_l;
            delta_encoder_r = encoder_r - prev_encoder_r;
            delta_encoder_h = encoder_h - prev_encoder_h;
            delta_ang = current_ang - prev_ang;

            // Calculate local changes
            delta_local_x = (delta_encoder_l + delta_encoder_r) / 2;
            delta_local_y = delta_encoder_h - (delta_ang * disM_encoderHtoCenter);

            // Convert local changes to global coordinates
           // current_ang = 0;
            delta_global_x = delta_local_x * Math.cos(current_ang) - delta_local_y * Math.sin(current_ang);
            delta_global_y = delta_local_x * Math.sin(current_ang) + delta_local_y * Math.cos(current_ang);

            // Update global positions
            global_xM += delta_global_x;
            global_yM += delta_global_y;

            // Display telemetry data
            telemetry.addData("x (meters)", global_xM);
            telemetry.addData("y (meters)", global_yM);
            telemetry.addData("Angle (degrees)", Math.toDegrees(current_ang));
            telemetry.update();

            // Update previous values for next loop iteration
            prev_encoder_l = encoder_l;
            prev_encoder_r = encoder_r;
            prev_encoder_h = encoder_h;
            prev_ang = current_ang;
        }
    }

    // Method to convert encoder ticks to meters
    private double encoderToMetres(int ticks) {
        double wheelDiameter = 0.038; // Diameter of the wheel in meters (3.8 cm)
        double ticksPerRevolution = 8192.0; // Number of encoder ticks per wheel revolution
        double circumference = wheelDiameter * Math.PI; // Circumference of the wheel in meters
        return (ticks / ticksPerRevolution) * circumference; // Convert ticks to meters
    }

    // Method to get the current angle from the IMU
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
