package org.firstinspires.ftc.teamcode.CurrentSeason.OperatorModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    }

    @Override
    public void main() {

    }


}
