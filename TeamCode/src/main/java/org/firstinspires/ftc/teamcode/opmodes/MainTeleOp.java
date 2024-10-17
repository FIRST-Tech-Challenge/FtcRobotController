package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.resourses.Utlities;

@TeleOp
public class MainTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        final double speedMultiplier = 0.40;
        initialize();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                robot.driveTrain.setSpeedMultiplier(speedMultiplier);
            } else{
                robot.driveTrain.setSpeedMultiplier(1);

            robot.odometry.updateOdometry();
            double target = Utlities.wrap(robot.anglePID.getTarget() + (-gamepad1.right_stick_x * robot.maxTurnDegPerSecond * robot.getDeltaTime() * speedMultiplier));
            robot.anglePID.setTarget(target);
            robot.anglePID.update(robot.odometry.getRobotAngle());
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
            telemetry.addData("Robot Angle",robot.odometry.getRobotAngle());
            telemetry.addData("Target Angle",target);
            telemetry.addData("Robot Power",robot.anglePID.getPower());
            telemetry.addData("Field Centric Driving 1:", robot.driveTrain.fieldCentricDriving);
            telemetry.update();

            }
        }
    }
}
