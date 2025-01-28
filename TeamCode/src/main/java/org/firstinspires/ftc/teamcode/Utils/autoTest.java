package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;

@Autonomous(name="Basic ATONUMS Autonomous", group="Linear Opmode")
public class autoTest extends LinearOpMode {

    final double robotCenterToArm = 10;
    Pose2d blueBasket = new Pose2d(64 - robotCenterToArm/Math.sqrt(2), 64 - robotCenterToArm/Math.sqrt(2),Math.toRadians(45));
    Pose2d redBasket = new Pose2d(-64 + robotCenterToArm/Math.sqrt(2), -64 + robotCenterToArm/Math.sqrt(2),Math.toRadians(225));
    Pose2d blueRobot2StartPosition = new Pose2d(-23.5, 64,Math.toRadians(270)).times(new Pose2d(0,0,Math.PI*2));
    Pose2d blueRobot1StartPosition = new Pose2d(23.5, 64,Math.toRadians(270));
    Pose2d redRobot1StartPosition = new Pose2d(-23.5,-64,Math.toRadians(90));
    Pose2d redRobot2StartPosition = new Pose2d(23.5,-64,Math.toRadians(90));
    Pose2d blueGamePiece1 = new Pose2d(-48 + robotCenterToArm, 26,Math.toRadians(180));
    Pose2d blueGamePiece2 = new Pose2d(-58 + robotCenterToArm, 26,Math.toRadians(180));
    Pose2d blueGamePiece3 = new Pose2d(-68 + robotCenterToArm, 26,Math.toRadians(180));
    Pose2d yellow1GamePiece1 = new Pose2d(68, 26 + robotCenterToArm,Math.toRadians(270));
    Pose2d yellow1GamePiece2 = new Pose2d(58, 26 + robotCenterToArm,Math.toRadians(270));
    Pose2d yellow1GamePiece3 = new Pose2d(48, 26 + robotCenterToArm,Math.toRadians(270));
    Pose2d redGamePiece1 = new Pose2d(68 - robotCenterToArm, -26, Math.toRadians(0));
    Pose2d redGamePiece2 = new Pose2d(58 - robotCenterToArm, -26, Math.toRadians(0));
    Pose2d redGamePiece3 = new Pose2d(48 - robotCenterToArm, -26, Math.toRadians(0));
    Pose2d yellow2GamePiece1 = new Pose2d(-68, -26 - robotCenterToArm,Math.toRadians(90));
    Pose2d yellow2GamePiece2 = new Pose2d(-58, -26 - robotCenterToArm,Math.toRadians(90));
    Pose2d yellow2GamePiece3 = new Pose2d(-48, -26 - robotCenterToArm,Math.toRadians(90));


    public void runOpMode() {
        Drivetrain.init(hardwareMap);
        ElevatorVertical.init(hardwareMap);
        ElevatorVerticalState elevatorState = ElevatorVerticalState.INTAKE;

        waitForStart();
        boolean flag = false;
        while (isStarted() & flag == false) {
            double start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 2500) {
                if (System.currentTimeMillis() - start < 1200) {
                    Drivetrain.driveByTime(0.3);
                }
//                if (System.currentTimeMillis() - start < 1000 && System.currentTimeMillis() - start > 800) {
//                    Drivetrain.driveByTime(0.1);
//                }
                if (System.currentTimeMillis() - start < 1600) {
                    elevatorState = ElevatorVerticalState.PUTSPECIMEN;
                    ElevatorVertical.operate(elevatorState, 0, 0);

                }
                if (System.currentTimeMillis() - start > 1700 && System.currentTimeMillis() - start < 1800) {
                    elevatorState = ElevatorVerticalState.SPECIMEN;
                    ElevatorVertical.operate(elevatorState, 0, 0);
                }

                flag = true;
            }


//        start = System.currentTimeMillis();
//        while(System.currentTimeMillis()-start<100){
//            Drivetrain.driveByTime(0.1);
//        }
        }
    }
}
