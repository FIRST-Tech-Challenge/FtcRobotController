package org.firstinspires.ftc.team13580.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp(name= "MyRobotCentric", group="Robot")
@Disabled
public class TCRobotCentric extends LinearOpMode{
    RobotHardware robot= new RobotHardware(this);

    @Override
    public void runOpMode(){
        double axial= 0;
        double lateral=0;
        double yaw=0;
        double arm=0;
        double handOffset=0;

        robot.init();

        waitForStart();
        while (opModeIsActive()){
            axial= gamepad1.left_stick_y;
            lateral= gamepad1.left_stick_x;
            yaw= gamepad1.right_stick_x;

            robot.driveRobotCentric(axial,lateral,yaw);

            if(gamepad1.right_bumper){
                handOffset += robot.HAND_SPEED;
            }else if(gamepad1.left_bumper){
                handOffset -= robot.HAND_SPEED;
            }
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            //robot.setHandPositions(handOffset);
        }
    }

}
