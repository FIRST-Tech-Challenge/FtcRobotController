package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class FieldCentric extends LinearOpMode{
    RobotHardware robot= new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException{
        robot.init();
        IMU imu= hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters( new RevHubOrientationOnRobot(
           RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
           RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        double axial=0;
        double lateral=0;
        double yaw=0;

        while(opModeIsActive()){
            axial= -gamepad1.left_stick_y;
            lateral= gamepad1.left_stick_x;
            yaw=gamepad1.right_stick_x;

            if(gamepad1.options){
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            robot.driveFieldCentric(rotX,rotY, yaw);


        }
    }
}
