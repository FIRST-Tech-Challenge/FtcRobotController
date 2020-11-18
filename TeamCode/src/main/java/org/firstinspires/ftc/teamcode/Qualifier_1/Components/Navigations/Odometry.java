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
    DcMotorEx odom1;
    DcMotorEx odom2;
    DcMotorEx odom3;
    int[] odomconst = {1,1,1};
    double ticks_per_inch = 1440/3.2;
    double robot_diameter = sqrt(619.84);
    double[] odom = new double[3];
    double xpos = 0;
    double ypos = 0;
    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();
    public void init(LinearOpMode opMode) {

        op = opMode;
        hardwareMap = op.hardwareMap;

        // Chassis encoders
        odom1 = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        odom3 = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        odom2 = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        // reset encoder count.
        odom1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odomconst[0]=-1;
        odomconst[1]=-1;
        odomconst[2]=1;
    }



    public double getAngle() {
        double angle = (odomconst[0]*odom1.getCurrentPosition() - odomconst[1]*odom2.getCurrentPosition()) / (ticks_per_inch *2* robot_diameter * Math.PI);
        angle *= 360;
        angle %= 360;
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public double[] track() {
       /* double[] data = {0, 0, 0};
        if (odom1.getCurrentPosition() != odom[0] || odom2.getCurrentPosition() != odom[1] || odom3.getCurrentPosition() != odom[2]) {
            double ydiff = (odom1.getCurrentPosition() - odom[0] + odom2.getCurrentPosition() - odom[1]) / (2 * ticks_per_inch);
            double xdiff = (odom3.getCurrentPosition() - odom[2]) / ticks_per_inch;
            odom[0] = odom1.getCurrentPosition();
            odom[1] = odom2.getCurrentPosition();
            odom[2] = odom3.getCurrentPosition();
            xpos += sin(getAngle() * Math.PI / 180) * ydiff + cos((getAngle() * Math.PI / 180)) * xdiff;
            ypos += cos(getAngle() * Math.PI / 180) * ydiff - sin((getAngle() * Math.PI / 180)) * xdiff;
            data[0] = xpos;
            data[1] = ypos;
            data[2] = getAngle();
            op.telemetry.addData("X", data[0]);
            op.telemetry.addData("Y", data[1]);
            op.telemetry.addData("odom1", odom1.getCurrentPosition());
            op.telemetry.addData("odom2",  odom2.getCurrentPosition());
            op.telemetry.addData("odom3", odom3.getCurrentPosition());
            op.telemetry.addData("angle", data[2]);
            op.telemetry.update();
            return data;*/
            double[] data = {0, 0, 0};
            if (odomconst[0]*odom1.getCurrentPosition() != odom[0] || odomconst[2]*odom2.getCurrentPosition() != odom[1] || odom3.getCurrentPosition() != odom[2]) {
                double ydiff = (odomconst[0]*odom1.getCurrentPosition() - odom[0] + odomconst[1]*odom2.getCurrentPosition() - odom[1]) / (2 * ticks_per_inch);
                double xdiff = (odomconst[2]*odom3.getCurrentPosition() - odom[2]) / ticks_per_inch;
                odom[0] = odomconst[0]*odom1.getCurrentPosition();
                odom[1] = odomconst[1]*odom2.getCurrentPosition();
                odom[2] = odomconst[2]*odom3.getCurrentPosition();
                xpos += sin(getAngle() * Math.PI / 180) * ydiff + cos((getAngle() * Math.PI / 180)) * xdiff;
                ypos += cos(getAngle() * Math.PI / 180) * ydiff - sin((getAngle() * Math.PI / 180)) * xdiff;
                data[0] = xpos;
                data[1] = ypos;
                data[2] = getAngle();
                /*op.telemetry.addData("X", data[0]);
                op.telemetry.addData("Y", data[1]);
                op.telemetry.addData("odom1", odomconst[0]*odom1.getCurrentPosition());
                op.telemetry.addData("odom2",  odomconst[1]*odom2.getCurrentPosition());
                op.telemetry.addData("odom3", odomconst[2]*odom3.getCurrentPosition());
                op.telemetry.addData("angle", data[2]);
                op.telemetry.update();*/
                return data;

        } else {
            data[0] = xpos;
            data[1] = ypos;
            data[2] = getAngle();
        }
        return data;
    }
}


