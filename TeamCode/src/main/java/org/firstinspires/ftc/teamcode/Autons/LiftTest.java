package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp
public class LiftTest extends LinearOpMode {
    public Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift();
        lift.init(hardwareMap);
        while(opModeIsActive()) {
            if(gamepad1.a) {
                lift.moveCC();
            }
            else if(gamepad1.b) {
                lift.moveCW();
            }
            else {
                lift.stop();
            }
        }
    }
}
