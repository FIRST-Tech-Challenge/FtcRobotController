package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "D&M LL AprilTag", group = "Sensor")
public class
SensorLimelightAprilTag extends LinearOpMode {


    private Limelight3A limelight;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);


        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotYaw = Math.round(orientation.getYaw(AngleUnit.DEGREES)) ;
            limelight.updateRobotOrientation(robotYaw);
            telemetry.addData("Yaw", robotYaw);

            telemetry.update();


            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {
                        double x = (100/(botpose_mt2.getPosition().x)) - 33; //inches
                        double y = (botpose_mt2.getPosition().y - 1.2) * 39.37; //inches
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();

        }
        limelight.stop();
    }
}

