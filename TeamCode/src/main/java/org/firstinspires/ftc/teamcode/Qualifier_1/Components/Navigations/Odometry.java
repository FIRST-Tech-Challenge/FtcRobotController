package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

public class Odometry {
    DcMotor odom1;
    DcMotor odom2;
    DcMotor odom3;
    DcMotorEx motorLeftFront;
    DcMotorEx motorRightFront;
    DcMotorEx motorLeftBack;
    DcMotorEx motorRightBack;
    double ticks_per_inch = 1440 / 2 * (3.14);
    double robot_diameter = 15;
    double[] odom = new double[3];
    double xpos = 0;
    double ypos = 0;
    int[] xd = new int[]{1, 0, -1, 0};
    int[] yd = new int[]{0, 1, 0, -1};
    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();
    public void init(LinearOpMode opMode) {

        op = opMode;
        hardwareMap = op.hardwareMap;

        // Chassis encoders
        odom1 = (DcMotor) hardwareMap.dcMotor.get("odom1");
        odom2 = (DcMotor) hardwareMap.dcMotor.get("odom2");
        odom3 = (DcMotor) hardwareMap.dcMotor.get("odom3");

        // Chassis Motor
        // reset encoder count.
        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    public double getAngle() {
        double angle = (odom1.getCurrentPosition() - odom2.getCurrentPosition()*-1) / (ticks_per_inch * robot_diameter * Math.PI);
        angle *= 360;
        angle %= 360;
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public double[] track() {
        double[] data = {0, 0, 0};
        if (odom1.getCurrentPosition() != odom[0] || odom2.getCurrentPosition()*-1 != odom[1] || odom3.getCurrentPosition() != odom[2]) {
            double ydiff = (odom1.getCurrentPosition() - odom[0] + odom2.getCurrentPosition()*-1 - odom[1]) / (2 * ticks_per_inch);
            double xdiff = (odom3.getCurrentPosition() - odom[2]) / ticks_per_inch;
            odom[0] = odom1.getCurrentPosition();
            odom[1] = odom2.getCurrentPosition() * -1;
            odom[2] = odom3.getCurrentPosition();
            xpos += sin(getAngle() * Math.PI / 180) * ydiff + cos(Math.PI / 2 - (getAngle() * Math.PI / 180)) * xdiff;
            ypos += cos(getAngle() * Math.PI / 180) * ydiff - sin(Math.PI / 2 - (getAngle() * Math.PI / 180)) * xdiff;
            data[0] = xpos;
            data[1] = ypos;
            data[2] = getAngle();
            op.telemetry.addData("X", data[0]);
            op.telemetry.addData("Y", data[1]);
            op.telemetry.addData("odom1", odom1.getCurrentPosition());
            op.telemetry.addData("odom2", -1 * odom2.getCurrentPosition());
            op.telemetry.addData("angle", data[2]);
            op.telemetry.update();
            return data;

        } else {
            data[0] = xpos;
            data[1] = ypos;
            data[2] = getAngle();
        }
        return data;
    }
}


