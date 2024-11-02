package org.firstinspires.ftc.teamcode.myUtil;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@Disabled
@TeleOp(name="Rotation Finder")
public class rotateFinder extends OpMode {

    Hardware r = new Hardware();
    @Override
    public void init() {
        r.initRobot(this);
        r.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double angle2 = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotation = gamepad1.right_stick_x;
        double power1 =  - rotation;
        double power2 =  - rotation;
        double power3 =  + rotation;
        double power4 =  + rotation;
        r.flm.setPower(power3);
        r.frm.setPower(power4);
        r.blm.setPower(power1);
        r.brm.setPower(power2);
        telemetry.addData("Angle", angle2);
        telemetry.addData("Ticks", r.flm.getCurrentPosition());
        telemetry.addData("TpD",r.flm.getCurrentPosition()/angle2);
        telemetry.update();
    }
}
