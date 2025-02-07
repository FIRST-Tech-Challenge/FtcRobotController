package org.firstinspires.ftc.team13580.TeleOp.FieldCentric1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name="Sec Field Centric", group="Robot")

public class b extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);

    @Override
    public void runOpMode(){
        double axial=0;
        double lateral=0;
        double yaw=0;

        robot.init();
        waitForStart();

        while (opModeIsActive()){
            axial=-gamepad1.left_stick_y;
            lateral=gamepad1.left_stick_x;
            yaw=gamepad1.right_stick_x;

            robot.driveFieldCentric(axial,lateral,yaw);
        }
    }
}
