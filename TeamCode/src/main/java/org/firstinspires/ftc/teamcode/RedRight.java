package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Red Right")
public class RedRight extends LinearOpMode {

    private MecanumEncoder drive = new MecanumEncoder(this);
    private String TESTBOT = "24342-RC";
    private String wifiSsid = "";
    private IMU imu;
    private Telemetry.Item debugOutout = null;
    @Override
    public void runOpMode()  throws InterruptedException
    {
        wifiSsid = WifiUtil.getConnectedSsid();


        // run once when init is pressed
        drive.initHardware(this.hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);
        //imu = drive.getImu();
        drive.resetYaw();
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        //debugOutout = telemetry.addData("Debug:", imu.getRobotYawPitchRollAngles().toString());

        // After we are done initializing our code, we wait for Start button.
        waitForStart();

        // Start button pressed, off we go.
        // move off the wall
        drive.driveForwardInches(0.4,2, 15.0);
        // move to in front of first sample and push it into observation
        drive.driveRightInches(0.4,21, 15.0);
        drive.driveForwardInches(0.4,51, 15.0);
        drive.driveRightInches(0.4,15, 15.0);
        drive.driveBackwardInches(0.4,48, 15.0);
        // go forward and then right and push the 2nd sample into observation
        drive.driveForwardInches(0.4,46, 15.0);
        drive.driveRightInches(0.4,7.5, 15.0);
        drive.driveBackwardInches(0.4,46, 15.0);
        // repeat the above^^^^
        drive.driveForwardInches(0.4,46, 15.0);
        drive.driveRightInches(0.4,8, 15.0);
        drive.driveBackwardInches(0.4,46, 15.0);



//        drive.driveForwardInches(0.4,52, 15.0);
//        drive.rotateCounterClockwise();

        drive.stop();
    }
}
