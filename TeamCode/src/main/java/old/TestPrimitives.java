package old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Primitives", group="Linear Opmode")

public class TestPrimitives extends LinearOpMode {
    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    boolean isBlue;
    double DEFAULT_POWER = 1; //was 0.6 for most
    double SLOW_POWER = 0.35; //was 0.35

    double TILE = 24 * 2.54;
    double ROBOT = 45;

    double RED_CROSS_FIELD_Y = -95;

    public void runOpMode() {

        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // optionally set starting position and orientation of the robot
        simpleTracking.setOrientationDegrees(0);
        simpleTracking.setPosition(0,0);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        simpleTracking.setModuleOrientation(robot);

        //robot.setPlacerUp();

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        double startTime = System.currentTimeMillis();

        while(opModeIsActive()) {
//            moveTo(0, 0, 180, 0.5, 2, 3000);
//            simplePathFollow.stop(robot);

            //robot.setPlacerUp();

            waitForButton();
        }

    }


    private void moveTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
        }
    }

    public void waitForButton () {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("IMU Heading: ", robot.getRobotHeading());
        }
    }

}
