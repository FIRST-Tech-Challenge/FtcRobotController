package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class TwoWDDrivetrain {

    public String ld1_name;
    public String rd1_name;

    public DcMotor ld1;
    public DcMotor rd1;

    public int LeftOffset = 1;
    public int RightOffset = 1;

    private HardwareMap hardware;

    public TwoWDDrivetrain(List<String> driveMotors, HardwareMap hardwareMap) {
        ld1_name = driveMotors.get(0);
        rd1_name = driveMotors.get(1);

        hardware = hardwareMap;

        ld1 = hardwareMap.dcMotor.get(ld1_name);
        rd1 = hardwareMap.dcMotor.get(rd1_name);
    }

    public void SetPower(double left, double right) {
        this.ld1.setPower(left);
        this.rd1.setPower(right);
    }

    /**
    * @param direction - Set to -1 to turn counterclockwise, otherwise set as 1
     */
    public void Rotate(double power, int direction) {
        SetPower(direction*power, -direction*power);
    }

    public void EvalGamepad(double x, double y) {
        double drive = y;
        double turn  =  -x;
        double left    = Range.clip(drive + turn, -100.0, 100.0) ;
        double right   = Range.clip(drive - turn, -100.0, 100.0) ;
        SetPower(left, -right);
    }
}
