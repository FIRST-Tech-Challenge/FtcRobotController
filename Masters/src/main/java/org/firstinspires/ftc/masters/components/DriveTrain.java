package org.firstinspires.ftc.masters.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.masters.CSCons;

public class DriveTrain extends Component{

    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    public void initializeHardware () {
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
    }
    public void drive(double x, double y, double t) {

        //        float threshold = 0.1;
        //
        //        if (Math.abs(t) < threshold)
        //        {
        //            t = 0;
        //        }

        if (Math.abs(y) < 0.2) {
            y = 0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0;
        }

        double leftFrontPower = y + x * CSCons.frontMultiplier + t;
        double leftRearPower = y - (x * CSCons.backMultiplier) + t;
        double rightFrontPower = y - x * CSCons.frontMultiplier - t;
        double rightRearPower = y + (x * CSCons.backMultiplier) - t;

        //if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightRearPower));

        leftFrontPower /= denominator;
        leftRearPower /= denominator;
        rightFrontPower /= denominator;
        rightRearPower /= denominator;
        //}

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    public void drive(double theta, double t){
        this.drive(Math.cos(theta), Math.sin(theta), t);
    }
}