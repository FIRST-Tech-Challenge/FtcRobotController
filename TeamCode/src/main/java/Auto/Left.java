package Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;

import static org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition.LEFT;
import static org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition.MIDDLE;

import androidx.annotation.VisibleForTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

public class Left {

    @Config
    @Autonomous(name = "Auto - Left", group = "Auto")
    public class AutoTest extends LinearOpMode {

        MotionHardware robot = new MotionHardware(this);
        VisionHardware vision = new VisionHardware(this);
        @Override
        public void runOpMode() throws InterruptedException {
            robot.init();
            vision.init();

            waitForStart();

            while(opModeIsActive()) {
                VisionHardware.PropPosition propPosition = vision.detectProp();

                // Center Strike Line
                // - Forward: 31.75
                // - Reverse: -44.75

                        //Drop off pixel
                        robot.moveRobot(.5, -44.75, 10);
                        //Pretend to drop pixel
                        sleep(1000);

                if (gamepad2.a) {
                    //wristServo.setPosition(0.6); // Adjust the position value as needed
                } else if (gamepad2.b) {
                    //wristServo.setPosition(0); // Adjust the position value for the center position
                }
                        //Backup and clear pixel
                        robot.moveRobot(.5, -5, 5);
                        sleep(1000);
                        //Turn to parking location
                        robot.turnRobot(MotionHardware.Direction.LEFT, 12, .5, 10);
                        sleep(1000);
                        //Park
                        robot.moveRobot(.5, 35, 10);


                        sleep(20);
                        break;
                }
            }
        }

    }
