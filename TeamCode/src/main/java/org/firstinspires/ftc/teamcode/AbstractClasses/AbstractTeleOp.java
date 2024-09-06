package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AbstractClasses.*;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;

import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.Pose;
import com.roboctopi.cuttlefishftcbridge.devices.*;
import com.roboctopi.cuttlefish.controller.*;
import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;

public abstract class AbstractTeleOp extends GamepadOpMode {

    PeppyFeetFiend robot = new PeppyFeetFiend(this);

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void mainLoop(){
        robot.chassis.setVec(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.left_stick_x));
    }

    public abstract AbstractRobot instantiateRobot();
}
