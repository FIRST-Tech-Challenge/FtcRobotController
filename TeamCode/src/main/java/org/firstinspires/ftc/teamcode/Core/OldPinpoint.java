package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

@Config
public class OldPinpoint {  // TODO: add junit
    GoBildaPinpointDriver odo;
    Pose2D pos;
    public double x, y, heading;
    public static double Xoffset, Yoffset;
    private RobotSettings robotSettings;



    public OldPinpoint(MechanumHardwareMap hwMap, RobotSettings robotSettings, boolean isAuto) {
        odo = hwMap.getOdo();
        this.robotSettings = robotSettings;
        Xoffset = -132.5; Yoffset = 14.075;
        odo.setOffsets(Xoffset, Yoffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections( GoBildaPinpointDriver.EncoderDirection.FORWARD,  GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // odo.resetPosAndIMU();

        // Pose2D pose2D = new Pose2D(DistanceUnit.METER, 0, 0.0, AngleUnit.DEGREES, 0.0);
        /* if (isAuto) {*/
        odo.setPosition(robotSettings.startPosState.getPose2D());
        //}

        update();
    }

    public boolean pinpointReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void update() {
        odo.update();
        pos = odo.getPosition();
        x = pos.getX(DistanceUnit.METER);
        y= pos.getY(DistanceUnit.METER);
        heading = pos.getHeading(AngleUnit.DEGREES);
    }

    public Pose2D getPos() {
        return pos;
    }

    public double getHeading() {
        return heading;
    }


    public double getX() {
        return x;
    }


    public double getY() {
        return y;
    }

    public void resetIMU() {
        if(robotSettings.alliance == RobotSettings.Alliance.RED){
            odo.setPosition(new Pose2D(DistanceUnit.METER, 1.64, 0.16, AngleUnit.DEGREES, 180));
        } else{
            odo.setPosition(new Pose2D(DistanceUnit.METER, 1.64, -0.16, AngleUnit.DEGREES, 180));
        }
    }

    public void resetPos() {
        odo.setPosition(new Pose2D(DistanceUnit.METER,1.2,0,AngleUnit.DEGREES,heading));
    }

    public double getGoalDistance() {
        return Math.sqrt(Math.pow((robotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - x), 2) + Math.pow((robotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - y), 2));
    }

    public void setPosition(Pose2D pose2D) {
        odo.setPosition(pose2D);
        odo.update();
    }

    /*public double getGoalHeading() {
        double error;
        if(MainAuto.ALLIANCE.equals("RED")) {
            error = RED_GOAL_POS.getHeading(AngleUnit.DEGREES) - heading;
        }
        else {
            error = BLUE_GOAL_POS.getHeading(AngleUnit.DEGREES) - heading;
        }
        if (Math.signum(error) != Math.signum(getHeadingErrorTrig())) {
            error = -error;
        }
        return error;

    }
*/
    public double getHeadingErrorTrig() {
        double targetAngle;
        targetAngle = Math.toDegrees(Math.atan2((robotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - y), (robotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - x)));

        double error = targetAngle - heading;

        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }
       /* error = -error;
        error = 360 - error;*/
        return error;
    }

}