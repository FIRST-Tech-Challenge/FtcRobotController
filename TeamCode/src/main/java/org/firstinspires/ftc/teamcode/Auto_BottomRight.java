package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HornetSquadObject;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;
@TeleOp(name = "HornetSquad: Auto Bottom Right", group = "Auto")
public class Auto_BottomRight extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);
    static final double FORWARD_SPEED = 0.2;
    static final double TURNSPEED = -0.2;
    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                robot.driveRobot(-FORWARD_SPEED, 0);
                sleep(6000);
                robot.driveRobot(0,0);

                robot.moveArmFullSpeed(RobotHardware.ARM_DOWN_POWER);
                sleep(2700);
                robot.stopArm();

                robot.driveRobot(FORWARD_SPEED, 0);
                sleep(4400);
                robot.driveRobot(0,TURNSPEED);
                sleep(5200);
                robot.driveRobot(FORWARD_SPEED,0);
                sleep(8000);
                robot.moveElbowToPosition(0.3);
                sleep(1000);
                robot.moveGrabberToPosition(RobotHardware.GRABBER_MIN);
                sleep(500);
                robot.moveElbowToPosition(-0.3);
                sleep(1000);
                robot.driveRobot(FORWARD_SPEED,0);
                sleep(1000);
                break;
            }
        }
    }
}

