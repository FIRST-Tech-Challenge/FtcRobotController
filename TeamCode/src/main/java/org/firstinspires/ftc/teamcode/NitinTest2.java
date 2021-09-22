
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.UpliftRobot2;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
@TeleOp (name = "NitinOp2", group = "Opmodes")
public class NitinTest2 extends UpliftTele {
    UpliftRobot2 robot;
    DcMotor tm;
    public void initHardware() {
        robot = new UpliftRobot2(this);
    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() {

        tm.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void exit() {

    }
}
