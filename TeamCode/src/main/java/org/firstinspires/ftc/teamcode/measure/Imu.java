package org.firstinspires.ftc.teamcode.measure;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Imu {
    private BNO055IMU bno055IMU;
    private Orientation orientation;

    public Imu(BNO055IMU bno055IMU) {
        this.bno055IMU = bno055IMU;
    }

    public void init(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        bno055IMU.initialize(parameters);
    }

    public float getAngularOrientation() {
        orientation = bno055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }
}
