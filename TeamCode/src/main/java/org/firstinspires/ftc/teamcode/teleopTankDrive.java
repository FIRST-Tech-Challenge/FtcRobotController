package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teleop Tank Drive")
public class teleopTankDrive extends LinearOpMode{
    zanehardware robot = new zanehardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //wait for start
        waitForStart();
        while (opModeIsActive()){
            //tank controls
            robot.leftMotorPower(gamepad1.left_stick_y);
            robot.rightMotorPower(gamepad1.right_stick_y);

            //spinner controls
            if(gamepad1.a){
                robot.Spinner.setPower(-0.3);
            } else if(gamepad1.y){
                robot.Spinner.setPower(0.3);
            } else {
                robot.Spinner.setPower(0);
            }
        }

    }

}
