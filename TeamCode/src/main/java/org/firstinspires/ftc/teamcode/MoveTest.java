package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


//@Disabled
@Autonomous(name="Move: Test", group="Test")
public class MoveTest extends OpMode{

    private int stateMachineFlow;
    MecanumDrive robot = new MecanumDrive();

    double time;
    private ElapsedTime     runtime = new ElapsedTime();
    /***********************************
     *
     * This program tests each direction the robot can move
     *
     ***********************************/
    @Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 25000;

        robot.init(hardwareMap);

        stateMachineFlow = 0;
    }

    @Override
    public void loop() {
        switch (stateMachineFlow) {
            case 0:
                runtime.reset();
                time = getRuntime();
                stateMachineFlow++;
                break;
            case 1:
                //move forward
                robot.linearDrive(.5,20);
                telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 2:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 3:
                //move backwards
                robot.linearDrive(.5,-20);
                telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 4:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 5:
                //move right
                robot.sideDrive(.5,20);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 6:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 7:
                //move left
                robot.sideDrive(.5,-20);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 8:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 9:
                //up and to the right
                robot.diagonalDrive(.5,20, DiagonalDirection.RIGHT);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 10:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 11:
                //down and to the left
                robot.diagonalDrive(.5,-20, DiagonalDirection.LEFT);
                //telemetry.addData("Rf",robot.getRFencoder());
                telemetry.addData("Lf",robot.getLFencoder());
                telemetry.addData("Rb",robot.getRBencoder());
                //telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 12:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 13:
                //up and to the left
                robot.diagonalDrive(.5, 20, DiagonalDirection.LEFT);
                telemetry.addData("Rf",robot.getRFencoder());
                //telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 14:
                //wait 3 seconds
                while (3 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 15:
                //down and to the right
                robot.diagonalDrive(.5, -20, DiagonalDirection.RIGHT);
                telemetry.addData("Rf",robot.getRFencoder());
                //telemetry.addData("Lf",robot.getLFencoder());
                //telemetry.addData("Rb",robot.getRBencoder());
                telemetry.addData("Lb",robot.getLBencoder());
                telemetry.update();
                time = runtime.time();
                stateMachineFlow++;
                break;
            case 16:
                //wait 5 seconds then stop
                while (5 > runtime.time() - time) {

                }
                stateMachineFlow++;
                break;
            case 17:
                stop();
        }
    }
}
