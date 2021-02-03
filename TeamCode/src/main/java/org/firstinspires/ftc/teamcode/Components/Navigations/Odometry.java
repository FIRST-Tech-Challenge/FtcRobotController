package org.firstinspires.ftc.teamcode.Components.Navigations;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.Components.Navigations.Navigation.*;


public class Odometry extends Thread {
    DcMotorEx odom1;
    DcMotorEx odom2;
    DcMotorEx odom3;
    int[] odomconst = {-1,1,-1};
    float ticks_per_inch = (float)(8640*2.54/38*Math.PI)*72/76;
    float robot_diameter = (float)sqrt(619.84);
    float[] odom = new float[3];
    private LinearOpMode op = null;
    private BNO055IMU imu;
    private Orientation             lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;


    //set true to enable imu vice versa
    final boolean enableIMU = true;

    public Odometry(LinearOpMode opMode) {

        op = opMode;

        // Chassis encoders
        odom1 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        odom3 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftBack");
        odom2 = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
        // reset encoder count.
        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastAngles  = new Orientation();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // make sure the imu gyro is calibrated before continuing.
        while (!op.isStopRequested() && !imu.isGyroCalibrated())
        {
            op.sleep(50);
            op.idle();
        }

        op.telemetry.addData("Mode", "waiting for start");
        op.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        op.telemetry.update();
    }
    public void run() {
        while(!isInterrupted()) {
            if(!getInVuforia()) {//getInVuforia
                float diff[]={odomconst[0]*(odom1.getCurrentPosition() - odom[0]),odomconst[1]*(odom2.getCurrentPosition() - odom[1]),odomconst[2]*(odom3.getCurrentPosition() - odom[2])};
                odom[0] += odomconst[0]*diff[0];
                odom[1] += odomconst[1]*diff[1];
                odom[2] += odomconst[2]*diff[2];
                float x =  (float)cos((getAngle() * Math.PI / 180));
                float y = (float)sin((getAngle() * Math.PI / 180));
                setYposition(getYposition()+(y * (diff[0]+diff[1])/(2*ticks_per_inch) + x * diff[2]/ticks_per_inch)*-1);
                setXposition(getXposition()+ (x * (diff[0]+diff[1])/(2*ticks_per_inch) + y * diff[2]/ticks_per_inch)*1);
                setAngle(getAngle());
                op.telemetry.addData("x",getXposition());
                op.telemetry.addData("y",getYposition());
                op.telemetry.addData("a",getAngle());
                op.telemetry.update();
            }
        }
    }
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //op.telemetry.addData("first angle: ", (int)angles.firstAngle);
        //op.telemetry.update();
        //op.sleep(1000);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle <= -180) //If the angle is -180, it should be 180, because they are at the same point. The acceptable angles are (-180, 180]
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }
}
