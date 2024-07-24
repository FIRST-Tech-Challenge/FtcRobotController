    package org.firstinspires.ftc.teamcode;  //place where the code is located

    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.IMU;

    @TeleOp(name = "IMUtest")
    public class IMUtest extends LinearOpMode{
        
        private IMU imu;
        
        @Override
        public void runOpMode() { // this code is for returning imu anges for testing purposes
            
            imu = hardwareMap.get(IMU.class, "imu");
            
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
            waitForStart();
            
            while (opModeIsActive()) {
                telemetry.addData("gyro", imu.getRobotYawPitchRollAngles());
                telemetry.update();
            }
        }
        
    }