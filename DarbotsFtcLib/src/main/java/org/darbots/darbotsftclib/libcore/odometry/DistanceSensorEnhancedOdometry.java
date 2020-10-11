package org.darbots.darbotsftclib.libcore.odometry;

import androidx.annotation.NonNull;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.DarbotsOnRobotSensor2D;
import org.darbots.darbotsftclib.libcore.sensors.distance_sensors.DarbotsRevDistanceSensor;
import org.darbots.darbotsftclib.libcore.templates.odometry.OdometryMethod;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotAsyncPositionTracker;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;

public class DistanceSensorEnhancedOdometry extends OdometryMethod {
    public static enum DistanceSensorOdometerSwitchType {
        ONLY_X,
        ONLY_Y,
        BOTH_XY,
        NONE
    };
    public static interface DistanceSensorOdometrySwitch{
        DistanceSensorOdometerSwitchType getSwitchType(double currentX, double currentY, double currentRotationZ, double squaredRotationZ);
    }
    public DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> frontDistanceSensor, leftDistanceSensor, backDistanceSensor, rightDistanceSensor;
    private OdometryMethod m_OriginalOdometryMethod;
    private double m_AngleErrorMargin = 3;
    private double c_HalfFieldSizeX, c_HalfFieldSizeY;
    public volatile DistanceSensorOdometerSwitchType overallSwitch = DistanceSensorOdometerSwitchType.BOTH_XY;
    public DistanceSensorOdometrySwitch callableSwitch = null;

    public DistanceSensorEnhancedOdometry(@NonNull OdometryMethod originalOdometryMethod, DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> frontDistanceSensor, DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> leftDistanceSensor, DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> backDistanceSensor, DarbotsOnRobotSensor2D<DarbotsRevDistanceSensor> rightDistanceSensor){
        this.m_OriginalOdometryMethod = originalOdometryMethod;
        this.frontDistanceSensor = frontDistanceSensor;
        this.leftDistanceSensor = leftDistanceSensor;
        this.backDistanceSensor = backDistanceSensor;
        this.rightDistanceSensor = rightDistanceSensor;
    }

    public DistanceSensorEnhancedOdometry(DistanceSensorEnhancedOdometry oldEnhancedOdometry){
        this.m_OriginalOdometryMethod = oldEnhancedOdometry.m_OriginalOdometryMethod;
        this.frontDistanceSensor = oldEnhancedOdometry.frontDistanceSensor;
        this.leftDistanceSensor = oldEnhancedOdometry.leftDistanceSensor;
        this.backDistanceSensor = oldEnhancedOdometry.backDistanceSensor;
        this.rightDistanceSensor = oldEnhancedOdometry.rightDistanceSensor;
        this.m_AngleErrorMargin = oldEnhancedOdometry.m_AngleErrorMargin;
        this.overallSwitch = oldEnhancedOdometry.overallSwitch;
        this.callableSwitch = oldEnhancedOdometry.callableSwitch;
    }

    public double getAngleErrorMargin(){
        return this.m_AngleErrorMargin;
    }

    public void setAngleErrorMargin(double angleErrorMargin){
        this.m_AngleErrorMargin = Math.abs(angleErrorMargin);
    }

    @NonNull
    public OdometryMethod getOriginalOdometryMethod(){
        return this.m_OriginalOdometryMethod;
    }

    public void setOriginalOdometryMethod(@NonNull OdometryMethod originalMethod){
        this.m_OriginalOdometryMethod = originalMethod;
    }

    @Override
    public void __trackStart() {
        this.m_OriginalOdometryMethod.setPositionTracker(this.getPositionTracker());
        this.m_OriginalOdometryMethod.__trackStart();
        this.c_HalfFieldSizeX = SkyStoneCoordinates.FIELD_SIZE_X / 2.0;
        this.c_HalfFieldSizeY = SkyStoneCoordinates.FIELD_SIZE_Y / 2.0;
    }

    public void __updateData(){
        this.frontDistanceSensor.Sensor.updateStatus();
        this.leftDistanceSensor.Sensor.updateStatus();
        this.backDistanceSensor.Sensor.updateStatus();
        this.rightDistanceSensor.Sensor.updateStatus();
    }

    @Override
    public void __trackLoop(double secondsSinceLastLoop) {
        if(!(this.getPositionTracker() instanceof RobotAsyncPositionTracker)){
            this.getPositionTracker().updateGyroProvider();
        }
        RobotPose2D beforePose2D = this.getPositionTracker().getCurrentPosition();
        double currentAngle = beforePose2D.getRotationZ();

        if(Math.abs(currentAngle % 90.0) > m_AngleErrorMargin){
            //angle error bigger than margin range, calling the original tracker to track the position
            this.m_OriginalOdometryMethod.__trackLoop(secondsSinceLastLoop);
            return;
        }

        double squaredAngle = XYPlaneCalculations.roundDegToSquare(currentAngle);
        boolean turned90 = squaredAngle == -90 || squaredAngle == 90;

        this.__updateData();
        double frontReading = this.frontDistanceSensor.Sensor.getDistanceInCM();
        double leftReading = this.leftDistanceSensor.Sensor.getDistanceInCM();
        double backReading = this.backDistanceSensor.Sensor.getDistanceInCM();
        double rightReading = this.rightDistanceSensor.Sensor.getDistanceInCM();

        boolean frontValid = frontReading != DarbotsRevDistanceSensor.DISTANCE_INVALID;
        boolean leftValid = leftReading != DarbotsRevDistanceSensor.DISTANCE_INVALID;
        boolean backValid = backReading != DarbotsRevDistanceSensor.DISTANCE_INVALID;
        boolean rightValid = rightReading != DarbotsRevDistanceSensor.DISTANCE_INVALID;

        double frontAngle = currentAngle;
        double leftAngle = XYPlaneCalculations.normalizeDeg(currentAngle + 90);
        double backAngle = XYPlaneCalculations.normalizeDeg(currentAngle - 180);
        double rightAngle = XYPlaneCalculations.normalizeDeg(currentAngle - 90);

        double distToPositiveXExt = DarbotsRevDistanceSensor.DISTANCE_INVALID, distToPositiveYExt = DarbotsRevDistanceSensor.DISTANCE_INVALID, distToNegativeXExt = DarbotsRevDistanceSensor.DISTANCE_INVALID, distToNegativeYExt = DarbotsRevDistanceSensor.DISTANCE_INVALID;
        if(frontValid){
            frontReading += frontDistanceSensor.OnRobotPosition.X;
            if(!turned90){
                double frontValue = Math.cos(Math.toRadians(frontAngle)) * frontReading;
                if(frontValue >= 0){
                    distToPositiveXExt = frontValue;
                }else{
                    distToNegativeXExt = -frontValue;
                }
            }else{
                double frontValue = Math.sin(Math.toRadians(frontAngle)) * frontReading;
                if(frontValue >= 0){
                    distToPositiveYExt = frontValue;
                }else{
                    distToNegativeYExt = -frontValue;
                }
            }
        }
        if(leftValid){
            leftReading += leftDistanceSensor.OnRobotPosition.Y;
            if(!turned90){
                double leftValue = Math.sin(Math.toRadians(leftAngle)) * leftReading;
                if(leftValue >= 0){
                    distToPositiveYExt = leftValue;
                }else{
                    distToNegativeYExt = -leftValue;
                }
            }else{
                double leftValue = Math.cos(Math.toRadians(leftAngle)) * leftReading;
                if(leftValue <= 0){
                    distToNegativeXExt = -leftValue;
                }else{
                    distToPositiveXExt = leftValue;
                }
            }
        }
        if(backValid){
            backReading -= backDistanceSensor.OnRobotPosition.X;
            if(!turned90){
                double backValue = Math.cos(Math.toRadians(backAngle)) * backReading;
                if(backReading <= 0){
                    distToNegativeXExt = -backValue;
                }else{
                    distToPositiveXExt = backValue;
                }
            }else{
                double backValue = Math.sin(Math.toRadians(backAngle)) * backReading;
                if(backValue <= 0){
                    distToNegativeYExt = -backValue;
                }else{
                    distToPositiveXExt = backValue;
                }
            }
        }
        if(rightValid){
            rightReading -= rightDistanceSensor.OnRobotPosition.Y;
            if(!turned90){
                double rightValue = Math.sin(Math.toRadians(rightAngle)) * rightReading;
                if(rightValue <= 0){
                    distToNegativeYExt = -rightValue;
                }else{
                    distToPositiveYExt = rightValue;
                }
            }else{
                double rightValue = Math.cos(Math.toRadians(rightAngle)) * rightReading;
                if(rightValue >= 0){
                    distToPositiveXExt = rightValue;
                }else{
                    distToNegativeXExt = -rightValue;
                }
            }
        }

        RobotPose2D afterPose = new RobotPose2D(beforePose2D);
        boolean positionShifted = false;
        //lets see which quadrant in the field we are in...
        DistanceSensorOdometerSwitchType switchType = this.overallSwitch;
        if(this.callableSwitch != null){
            DistanceSensorOdometerSwitchType calculatedType = this.callableSwitch.getSwitchType(beforePose2D.X,beforePose2D.Y, beforePose2D.getRotationZ(), squaredAngle);
            if((switchType == DistanceSensorOdometerSwitchType.NONE) || (switchType == DistanceSensorOdometerSwitchType.ONLY_X && calculatedType == DistanceSensorOdometerSwitchType.ONLY_Y) || (switchType == DistanceSensorOdometerSwitchType.ONLY_Y && calculatedType == DistanceSensorOdometerSwitchType.ONLY_X) || calculatedType == DistanceSensorOdometerSwitchType.NONE) {
                switchType = DistanceSensorOdometerSwitchType.NONE;
            }else if(switchType != DistanceSensorOdometerSwitchType.BOTH_XY){
                //switchType = switchType;
            }else{
                switchType = calculatedType;
            }
        }

        if(beforePose2D.X >= 0 && beforePose2D.Y >= 0){
            //first quadrant
            if (distToPositiveXExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_X || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.X = this.c_HalfFieldSizeX - distToPositiveXExt;
            }
            if (distToPositiveYExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_Y || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.Y = this.c_HalfFieldSizeY - distToPositiveYExt;
            }
        }else if(beforePose2D.X >= 0 && beforePose2D.Y < 0){
            //forth quadrant
            if (distToPositiveXExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_X || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.X = this.c_HalfFieldSizeX - distToPositiveXExt;
            }
            if (distToNegativeYExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_Y || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.Y = -(this.c_HalfFieldSizeY - distToNegativeYExt);
            }
        }else if(beforePose2D.X < 0 && beforePose2D.Y >= 0){
            //second quadrant
            if (distToNegativeXExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_X || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.X = -(this.c_HalfFieldSizeX - distToNegativeXExt);
            }
            if (distToPositiveYExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_Y || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)) {
                positionShifted = true;
                afterPose.Y = this.c_HalfFieldSizeY - distToPositiveYExt;
            }
        }else{ //beforePose2D.X < 0 && beforePose2D.Y < 0
            //quadrant 3
            if(distToNegativeXExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_X || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)){
                positionShifted = true;
                afterPose.X = -(this.c_HalfFieldSizeX - distToNegativeXExt);
            }
            if(distToNegativeYExt != DarbotsRevDistanceSensor.DISTANCE_INVALID && (switchType == DistanceSensorOdometerSwitchType.ONLY_Y || switchType == DistanceSensorOdometerSwitchType.BOTH_XY)){
                positionShifted = true;
                afterPose.Y = -(this.c_HalfFieldSizeY - distToNegativeYExt);
            }
        }

        if(positionShifted){
            this.m_OriginalOdometryMethod.__trackLoop_NoActualPositionShift(secondsSinceLastLoop);
            RobotPose2D shiftedPose = XYPlaneCalculations.getRelativePosition(beforePose2D,afterPose);
            RobotVector2D speed = new RobotVector2D(shiftedPose.X / secondsSinceLastLoop, shiftedPose.Y / secondsSinceLastLoop, 0);
            this.getPositionTracker().setCurrentPosition(afterPose);
            this.getPositionTracker().setCurrentVelocityVector(speed);
        }else{
            this.m_OriginalOdometryMethod.__trackLoop(secondsSinceLastLoop);
        }
    }

    @Override
    public void __trackLoop_NoActualPositionShift(double secondsSinceLastLoop) {
        this.m_OriginalOdometryMethod.__trackLoop_NoActualPositionShift(secondsSinceLastLoop);
    }
}
