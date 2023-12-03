package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestingWade extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, this, telemetry, false, true);
        robot.setUpDrivetrainMotors();

        // Calculate distances

        /*
        boolean isRedAlliance = true;
        int HORIZONTAL_TOTAL_BEFORE_CHUNKING = 44;
        int VERTICAL_TOTAL = 73;
        int polarity = -1;
        int vertical1 = 0;
        int horizontal2 = 18;
        int horizontal3 = 0;
        int vertical4 = 0; //adjust for left
        int horizontal5 = 26;
        int vertical6 = VERTICAL_TOTAL + vertical1 - vertical4;
        int horizontal7 = HORIZONTAL_TOTAL_BEFORE_CHUNKING - 15;
        int count = 0;
        */

        waitForStart();

        while (opModeIsActive()) {

            robot.straightBlocking(6, false, 0.7);

            /*
//            while (count < 5) {
                robot.straightBlockingFixHeading(horizontal2 + horizontal5, true, 0.8); //go forward FAST
                this.sleep(5000);
                robot.straightBlockingFixHeading(horizontal2 + horizontal5, false, 0.8); //go forward FAST
                //robot.straightBlockingFixHeading(horizontal5, false, 0.7); //go forward & around marker
                robot.setHeading(90 * polarity, 0.7); //turn
                robot.straightBlockingFixHeading(vertical6, false, 1);
                robot.setHeading(90 * polarity, 0.7);
                robot.mecanumBlocking(horizontal7, false, 0.5); //mecanum directly in front of board left if blue
                robot.setHeading(90 * polarity, 0.7);
                robot.setHeading(90 * polarity, 0.7);
                robot.mecanumBlocking(horizontal7, true, 0.5); //mecanum directly in front of board left if blue
                robot.setHeading(90 * polarity, 0.7);
                robot.straightBlockingFixHeading(vertical6, true, 1);
                robot.setHeading(0, 0.7); //turn
                robot.straightBlockingFixHeading(horizontal5, true, 1); //go forward & around marker
                robot.straightBlockingFixHeading(horizontal2, true, 1); //go forward & around marker
                count++;
                 */
//            }
            break;
        }
    }
}
