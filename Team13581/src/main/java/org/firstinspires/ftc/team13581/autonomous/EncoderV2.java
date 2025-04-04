package org.firstinspires.ftc.team13581.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="EncoderV2", group="Robot")
public class EncoderV2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        waitForStart();
        // Step  through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        robot.setvSlideDrivePosition();
        robot.vSlidePosition = robot.VSLIDE_START_POSITION;
        robot.setvSlideDrivePosition();
        robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
        robot.setvSlideDrivePosition();
        robot.encoderDrive(0.55, -18.5, -18.5, -18.5, -18.5, 5);
        sleep(900);
        robot.intake2.setPosition(1.0 / 9);
        sleep(1275);
        robot.intake2.setPosition(0);
        sleep(100);
        robot.vSlidePosition = robot.VSLIDE_START_POSITION;
        robot.setvSlideDrivePosition();
        sleep(50000);

        // Preload sample is scored, use this in all autos if possible.
















































        //while (opModeIsActive() && getRuntime() < 1) {
           // robot.intake.setPower(robot.INTAKE_COLLECT);

        //resetRuntime();




    }
}