package org.firstinspires.ftc.teamcode.CurrentSeason.OperatorModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;

@TeleOp(name="TeleOperator")
public class NikitaSucks extends AbstractTeleOp {
    PeppyFeetFiend robot;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new PeppyFeetFiend(this);
        return robot;
    }

    @Override
    public void onInit() {
        super.onInit();
        instantiateRobot();
    }

    @Override
    public void main() {
    }

    @Override
    public void mainLoop() {
        super.drive.setVec(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
        robot.driverLoop();
    }


}
