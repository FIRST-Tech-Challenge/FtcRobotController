/*
package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Teleop.Teleop;

public class TipAngle extends HWMap {

    Orientation lastAngle = new Orientation();
    private double globalPitchAngle;

    public enum TIP {
        TIPPING, NOT_TIPPING, ON_STACKS
    }

    private TIP tip = TIP.NOT_TIPPING;

    public TipAngle(Telemetry telemetry, HardwareMap hardwareMap){
        super(telemetry, hardwareMap);
    }

    public double getPitch() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float deltaPitchAngle = angles.thirdAngle - lastAngle.thirdAngle;//This is subtracting roll angle
        // It's going to record angles between -180 and 180
        globalPitchAngle += deltaPitchAngle;
        lastAngle = angles;
        return globalPitchAngle;
    }

    public void activateTip() {
        if (getPitch() <= 75) {
            //Here it checks if the tip angle exceeds 8 degrees
            tip = TIP.TIPPING;
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftFrontMotor.setPower(1.0);
            rightFrontMotor.setPower(1.0);
        } else if (getPitch() >= 100) {
            tip = TIP.TIPPING;
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightBackMotor.setPower(1.0);
            leftBackMotor.setPower(1.0);
        } else {
            tip = TIP.NOT_TIPPING;
            rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public TIP getTIP(){
        return tip;
    }

    public void setTIP(TIP tip){
        this.tip = tip;
    }


}
*/
