package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MainTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
//            robot.IMU_Update();
//            robot.anglePID.setTarget(robot.anglePID.getTarget() + (gamepad1.right_stick_x * robot.turningInputConstant * robot.getDeltaTime()));
//            robot.anglePID.update(robot.radiansToDegrees(robot.getRobotAngle()));
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x,robot.getRobotAngle());
            }
           
        }
    }

