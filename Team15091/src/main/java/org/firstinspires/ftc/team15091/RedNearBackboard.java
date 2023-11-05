package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RedNearBackboard", preselectTeleOp = "Gamepad")
public class RedNearBackboard extends AutonomousBase{
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setTargetPosition(800);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.8);
        sleep(500);
    }
}

