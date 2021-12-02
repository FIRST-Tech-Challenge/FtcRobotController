package org.firstinspires.ftc.teamcode.Configs.oldConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Turning", group="Pushbot")
public class Imu2 extends BaseOpMode {
    public Hardware2 robot = new Hardware2(true);  // We are using encoders, so pass it ????
    private ElapsedTime runtime = new ElapsedTime();
    //instantiating variable imu of type BNO055IMU
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        //write all run code here
        if (opModeIsActive()) {
            robot.turnRight(90, 0.1);
        }
    }

}