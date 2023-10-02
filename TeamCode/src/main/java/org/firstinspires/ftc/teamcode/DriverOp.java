package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DriverOperationMode")
public class DriverOp extends RobotOpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void robotLoop() {
        gamepadMoveRobot();
    }
}
