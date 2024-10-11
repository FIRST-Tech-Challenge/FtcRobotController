package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.KalmanFilter;
import org.firstinspires.ftc.teamcode.subsystems.PrimaryLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.SparkOdo;
import org.firstinspires.ftc.teamcode.subsystems.ThreeEncoderLocalizer;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

@TeleOp(name="KalmanFilterTest",group = "Subsystem Tests")
public class KalmanFilterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Mecanum drive = new Mecanum(hardwareMap);

        KalmanFilter filter = new KalmanFilter(0.1);

        SparkOdo spark = new SparkOdo(hardwareMap);
        ThreeEncoderLocalizer threeWheel = new ThreeEncoderLocalizer(hardwareMap);

        PrimaryLocalizer pLocalizer = new PrimaryLocalizer(new LocalizerInterface[]{
           spark,
           threeWheel
        });

        waitForStart();
        long lastTimestamp = System.currentTimeMillis();
        while(!isStopRequested()){

            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //Kalman Filter Stuff

            //Set up new time step
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTimestamp) / 1000.0;
            // Update the Kalman filter's transition matrix based on the new dt
            filter.setTimeStep(dt);

            // Perform prediction
            filter.predict();
            Pose2d sparkData = spark.getPosition();
            Pose2d threeData = threeWheel.getPosition();
            double [][] data = new double[][]{
                new double[] {sparkData.position.x, sparkData.position.y, Math.toDegrees(sparkData.heading.toDouble())},
                new double[] {threeData.position.x, threeData.position.y, Math.toDegrees(threeData.heading.toDouble())},
//                new double[] {0,0,0} //Future Limelight comptabilitiy?
            };
            // Update the filter with sensor data
            filter.updateFromSensors(data);



            //Control Localizer (Actual)
            telemetry.addLine("Average Data: ");
            telemetry.addLine(pLocalizer.toString() + "\n");


            telemetry.addLine("Kalman Filter results");
            telemetry.addLine(filter.getStateString());
            telemetry.update();
        }
    }
}
