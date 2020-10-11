package org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;

import java.util.LinkedList;
import java.util.List;

public class XYPlaneCalculations {
    public static final double CONST_180_OVER_PI = 180.0 / Math.PI;
    public static final double CONST_PI_OVER_180 = Math.PI / 180;
    public static final double[] ORIGIN_POINT_ARRAY = {0,0};
    public static final RobotPoint2D ORIGIN_POINT = new RobotPoint2D(0,0);
    public static final double VERY_SMALL = 0.00001;
    public static final double INCH_PER_CM = 0.393701;

    public static double[] rotatePointAroundFixedPoint_Deg(double[] point, double[] fixedPoint, double counterClockwiseAng) {
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = Math.toRadians(counterClockwiseAng);
        double sinDeltaAng = Math.sin(deltaAng);
        double cosDeltaAng = Math.cos(deltaAng);
        double newX = relativeX * cosDeltaAng - relativeY * sinDeltaAng;
        double newY = relativeX * sinDeltaAng + relativeY * cosDeltaAng;
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
    }

    public static RobotPoint2D rotatePointAroundFixedPoint_Deg(RobotPoint2D point, RobotPoint2D fixedPoint, double CCWAng){
        double[] pointArr = {point.X,point.Y};
        double[] fixedPointArr = {fixedPoint.X, fixedPoint.Y};
        double[] rotatedAng = rotatePointAroundFixedPoint_Deg(pointArr,fixedPointArr,CCWAng);
        return new RobotPoint2D(rotatedAng[0],rotatedAng[1]);
    }

    public static double[] rotatePointAroundFixedPoint_Rad(double[] point, double[] fixedPoint, double counterClockwiseAng){
        double relativeY = point[1] - fixedPoint[1], relativeX = point[0] - fixedPoint[0];
        double deltaAng = counterClockwiseAng;
        double sinDeltaAng = Math.sin(deltaAng);
        double cosDeltaAng = Math.cos(deltaAng);
        double newX = relativeX * cosDeltaAng - relativeY * sinDeltaAng;
        double newY = relativeX * sinDeltaAng + relativeY * cosDeltaAng;
        double[] result = {newX + fixedPoint[0], newY + fixedPoint[1]};
        return result;
    }

    public static RobotPoint2D rotatePointAroundFixedPoint_Rad(RobotPoint2D point, RobotPoint2D fixedPoint, double CCWAng){
        double[] pointArr = {point.X,point.Y};
        double[] fixedPointArr = {fixedPoint.X, fixedPoint.Y};
        double[] rotatedAng = rotatePointAroundFixedPoint_Rad(pointArr,fixedPointArr,CCWAng);
        return new RobotPoint2D(rotatedAng[0],rotatedAng[1]);
    }

    public static RobotPoint2D getRelativePosition(RobotPose2D PerspectiveOrigin, RobotPoint2D Target){
        //First step - move the Perspective Origin to the Origin of the Axis.
        double[] targetPoint = {Target.X - PerspectiveOrigin.X,Target.Y - PerspectiveOrigin.Y};
        //Second step - rotate the targetPoint so that the coordinate system (X and Z scalars) of the Perspective Origin overlaps with the Field Coordinate.
        //We are basically rotating field coordinate here.
        double[] origin = {0,0};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(targetPoint,origin,-PerspectiveOrigin.getRotationZ());

        return new RobotPoint2D(rotatedTargetPoint[0],rotatedTargetPoint[1]);
    }

    public static RobotPoint2D getAbsolutePosition(RobotPose2D PerspectiveOrigin, RobotPoint2D RelativePosition){
        //First Step - rotate the coordinates back.
        double[] origin = {0,0};
        double[] relativeTargetPoint = {RelativePosition.X,RelativePosition.Y};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(relativeTargetPoint,origin,PerspectiveOrigin.getRotationZ());
        //Second Step - move the PerspectiveOrigin back to the Absolute Point on the Field.
        double[] movedTargetPoint = {rotatedTargetPoint[0] + PerspectiveOrigin.X,rotatedTargetPoint[1] + PerspectiveOrigin.Y};

        return new RobotPoint2D(movedTargetPoint[0],movedTargetPoint[1]);
    }

    public static RobotPose2D getRelativePosition(RobotPose2D PerspectiveOrigin, RobotPose2D Target){
        //First step - move the Perspective Origin to the Origin of the Axis.
        double[] targetPoint = {Target.X - PerspectiveOrigin.X,Target.Y - PerspectiveOrigin.Y};
        //Second step - rotate the targetPoint so that the coordinate system (X and Z scalars) of the Perspective Origin overlaps with the Field Coordinate.
        //We are basically rotating field coordinate here.
        double[] origin = {0,0};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(targetPoint,origin,-PerspectiveOrigin.getRotationZ());
        //Third step - calculate relative delta Rotation Y;
        double deltaRotZ = normalizeDeg(Target.getRotationZ() - PerspectiveOrigin.getRotationZ());

        return new RobotPose2D(rotatedTargetPoint[0],rotatedTargetPoint[1],deltaRotZ);
    }

    public static RobotPose2D getAbsolutePosition(RobotPose2D PerspectiveOrigin, RobotPose2D RelativePosition){
        //First Step - calculate absolute Rotation Y.
        double absRotZ = normalizeDeg(RelativePosition.getRotationZ() + PerspectiveOrigin.getRotationZ());
        //Second Step - rotate the coordinates back.
        double[] origin = {0,0};
        double[] relativeTargetPoint = {RelativePosition.X,RelativePosition.Y};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(relativeTargetPoint,origin,PerspectiveOrigin.getRotationZ());
        //Third Step - move the PerspectiveOrigin back to the Absolute Point on the Field.
        double[] movedTargetPoint = {rotatedTargetPoint[0] + PerspectiveOrigin.X,rotatedTargetPoint[1] + PerspectiveOrigin.Y};

        return new RobotPose2D(movedTargetPoint[0],movedTargetPoint[1],absRotZ);
    }

    public static RobotPose2D getAbsolutePosition(double robotRotationInDegOnField, RobotPoint2D relativeObjectOnBot, RobotPoint2D relativeObjectOnField){
        robotRotationInDegOnField = XYPlaneCalculations.normalizeDeg(robotRotationInDegOnField);
        RobotPoint2D relativeObjectOnBot_RotatedToField = rotatePointAroundFixedPoint_Deg(relativeObjectOnBot,XYPlaneCalculations.ORIGIN_POINT,robotRotationInDegOnField);
        return new RobotPose2D(relativeObjectOnField.X - relativeObjectOnBot_RotatedToField.X, relativeObjectOnField.Y - relativeObjectOnBot_RotatedToField.Y, robotRotationInDegOnField);
    }

    public static RobotVector2D getRelativePosition(RobotVector2D PerspectiveOrigin, RobotVector2D Target){
        //First step - move the Perspective Origin to the Origin of the Axis.
        double[] targetPoint = {Target.X - PerspectiveOrigin.X,Target.Y - PerspectiveOrigin.Y};
        //Second step - rotate the targetPoint so that the coordinate system (X and Z scalars) of the Perspective Origin overlaps with the Field Coordinate.
        //We are basically rotating field coordinate here.
        double[] origin = {0,0};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(targetPoint,origin,-PerspectiveOrigin.getRotationZ());
        //Third step - calculate relative delta Rotation Y;
        double deltaRotZ = Target.getRotationZ() - PerspectiveOrigin.getRotationZ();

        return new RobotVector2D(rotatedTargetPoint[0],rotatedTargetPoint[1],deltaRotZ);
    }

    public static RobotVector2D getAbsolutePosition(RobotVector2D PerspectiveOrigin, RobotVector2D RelativePosition){
        //First Step - calculate absolute Rotation Y.
        double absRotZ = RelativePosition.getRotationZ() + PerspectiveOrigin.getRotationZ();
        //Second Step - rotate the coordinates back.
        double[] origin = {0,0};
        double[] relativeTargetPoint = {RelativePosition.X,RelativePosition.Y};
        double[] rotatedTargetPoint = rotatePointAroundFixedPoint_Deg(relativeTargetPoint,origin,PerspectiveOrigin.getRotationZ());
        //Third Step - move the PerspectiveOrigin back to the Absolute Point on the Field.
        double[] movedTargetPoint = {rotatedTargetPoint[0] + PerspectiveOrigin.X,rotatedTargetPoint[1] + PerspectiveOrigin.Y};

        return new RobotVector2D(movedTargetPoint[0],movedTargetPoint[1],absRotZ);
    }

    public static double chooseAngleFromRange(double[] angleList, double angleSmallestRange, double angleBiggestRange) {
        for(int i=0;i<angleList.length;i++) {
            if(angleList[i] >= angleSmallestRange && angleList[i] <= angleBiggestRange) {
                return angleList[i];
            }
        }
        return angleList[0];
    }

    public static double normalizeRad(double Rad) {
        double PIT2 = Math.PI * 2;
        double tempDeg = Rad % PIT2;
        if(tempDeg >= Math.PI){
            tempDeg -= PIT2;
        }else if(tempDeg < Math.PI){
            tempDeg += PIT2;
        }
        return tempDeg;
    }

    public static float normalizeRad(float Rad){
        return (float) normalizeRad((double) Rad);
    }

    public static double normalizeDeg(double Deg) {
        double tempDeg = Deg % 360;
        if(tempDeg >= 180){
            tempDeg -= 360;
        }else if(tempDeg < -180){
            tempDeg += 360;
        }
        return tempDeg;
    }

    public static double roundDegToSquare(double Deg){
        if(Deg >= 135 && Deg < -135){
            return -180;
        }else if(Deg >= -135 && Deg < -45){
            return -90;
        }else if(Deg >= -45 && Deg < 45){
            return 0;
        }else{ //Deg >= 45 && Deg < 135
            return 90;
        }
    }

    public static float normalizeDeg(float Deg) {
        return (float) normalizeDeg((double) Deg);
    }

    public static float roundDegToSquare(float Deg){
        if(Deg >= 135.0f && Deg < -135.0f){
            return -180.0f;
        }else if(Deg >= -135.0f && Deg < -45.0f){
            return -90.0f;
        }else if(Deg >= -45.0f && Deg < 45.0f){
            return 0.0f;
        }else{ //Deg >= 45.0f && Deg < 135.0f
            return 90.0f;
        }
    }

    public static boolean isPointAtCircleEdge(RobotPoint2D circleCenter, double circleRadius, RobotPoint2D point){
        if(point.distanceTo(circleCenter) <= VERY_SMALL){
            return true;
        }else{
            return false;
        }
    }

    public static LinkedList<RobotPoint2D> verticalLineCircleIntersections(RobotPoint2D circleCenter, double circleRadius, double fixedX){
        double x1 = fixedX - circleCenter.X;
        double ySquared = Math.pow(circleRadius,2) - Math.pow(x1,2);
        LinkedList<RobotPoint2D> allPoints = new LinkedList<RobotPoint2D>();
        if(ySquared < 0){
            return allPoints;
        }
        if(ySquared == 0){
            RobotPoint2D root = new RobotPoint2D(fixedX,circleCenter.Y);
            allPoints.add(root);
        }else{
            double sqrtYSquared = Math.sqrt(ySquared);
            RobotPoint2D root1 = new RobotPoint2D(fixedX,sqrtYSquared + circleCenter.Y);
            allPoints.add(root1);
            RobotPoint2D root2 = new RobotPoint2D(fixedX,-sqrtYSquared + circleCenter.Y);
            allPoints.add(root2);
        }
        return allPoints;
    }

    public static LinkedList<RobotPoint2D> horizontalLineCircleIntersections(RobotPoint2D circleCenter, double circleRadius, double fixedY){
        double y1 = fixedY - circleCenter.Y;
        double xSquared = Math.pow(circleRadius,2) - Math.pow(y1,2);
        LinkedList<RobotPoint2D> allPoints = new LinkedList<RobotPoint2D>();
        if(xSquared < 0){
            return allPoints;
        }
        if(xSquared == 0){
            RobotPoint2D root = new RobotPoint2D(circleCenter.X,fixedY);
            allPoints.add(root);
        }else{
            double sqrtXSquared = Math.sqrt(xSquared);
            RobotPoint2D root1 = new RobotPoint2D(sqrtXSquared + circleCenter.X,fixedY);
            allPoints.add(root1);
            RobotPoint2D root2 = new RobotPoint2D(-sqrtXSquared + circleCenter.X,fixedY);
            allPoints.add(root2);
        }
        return allPoints;
    }

    public static LinkedList<RobotPoint2D> lineSegmentCircleIntersections(RobotPoint2D circleCenter, double circleRadius, RobotPoint2D linePoint1, RobotPoint2D linePoint2){
        RobotPoint2D boundingBoxMin = new RobotPoint2D(
                Math.min(linePoint1.X,linePoint2.X),
                Math.min(linePoint1.Y,linePoint2.Y)
        );
        RobotPoint2D boundingBoxMax = new RobotPoint2D(
                Math.max(linePoint1.X,linePoint2.X),
                Math.max(linePoint1.Y,linePoint2.Y)
        );

        if(linePoint1.X == linePoint2.X || linePoint1.Y == linePoint2.Y){
            LinkedList<RobotPoint2D> resultArray = null;
            if(linePoint1.X == linePoint2.X && linePoint1.Y == linePoint2.Y){
                resultArray = new LinkedList<RobotPoint2D>();
                if(isPointAtCircleEdge(circleCenter,circleRadius,linePoint1)){
                    resultArray.add(linePoint1);
                }
            }else if(linePoint1.X == linePoint2.X){
                resultArray = verticalLineCircleIntersections(circleCenter,circleRadius,linePoint1.X);
            }else { //linePoint1.Y == linePoint2.Y
                resultArray = horizontalLineCircleIntersections(circleCenter,circleRadius,linePoint1.Y);
            }
            LinkedList<RobotPoint2D> allPoints = new LinkedList<RobotPoint2D>();
            for(RobotPoint2D i : resultArray){
                if(isInBoundingBox(i,boundingBoxMin,boundingBoxMax)){
                    allPoints.add(i);
                }
            }
            return allPoints;
        }

        double x1 = linePoint1.X - circleCenter.X;
        double y1 = linePoint1.Y - circleCenter.Y;

        double m1 = (linePoint2.Y - linePoint1.Y) / (linePoint2.X - linePoint1.X);
        double quadraticA = 1.0 + Math.pow(m1,2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);
        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(circleRadius,2);
        double quadraticDelta = Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC);

        LinkedList<RobotPoint2D> allPoints = new LinkedList<RobotPoint2D>();
        if(quadraticDelta > 0) {
            double xRoot1 = (-quadraticB + Math.sqrt(quadraticDelta)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.X;
            yRoot1 += circleCenter.Y;
            RobotPoint2D root1 = new RobotPoint2D(xRoot1, yRoot1);
            if (isInBoundingBox(root1, boundingBoxMin, boundingBoxMax)) {
                allPoints.add(root1);
            }
            double xRoot2 = (-quadraticB - Math.sqrt(quadraticDelta)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.X;
            yRoot2 += circleCenter.Y;
            RobotPoint2D root2 = new RobotPoint2D(xRoot2,yRoot2);
            if(isInBoundingBox(root2,boundingBoxMin,boundingBoxMax)){
                allPoints.add(root2);
            }
        }else if(quadraticDelta == 0){
            double xRoot1 = (-quadraticB) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.X;
            yRoot1 += circleCenter.Y;
            RobotPoint2D root1 = new RobotPoint2D(xRoot1, yRoot1);
            if (isInBoundingBox(root1, boundingBoxMin, boundingBoxMax)) {
                allPoints.add(root1);
            }
        }
        return allPoints;
    }

    public static LinkedList<RobotPoint2D> lineCircleIntersections(RobotPoint2D circleCenter, double circleRadius, RobotPoint2D linePoint1, RobotPoint2D linePoint2){
        if(linePoint1.X == linePoint2.X || linePoint1.Y == linePoint2.Y){
            LinkedList<RobotPoint2D> resultArray = null;
            if(linePoint1.X == linePoint2.X && linePoint1.Y == linePoint2.Y){
                resultArray = new LinkedList<RobotPoint2D>();
                if(isPointAtCircleEdge(circleCenter,circleRadius,linePoint1)){
                    resultArray.add(linePoint1);
                }
            }else if(linePoint1.X == linePoint2.X){
                resultArray = verticalLineCircleIntersections(circleCenter,circleRadius,linePoint1.X);
            }else { //linePoint1.Y == linePoint2.Y
                resultArray = horizontalLineCircleIntersections(circleCenter,circleRadius,linePoint1.Y);
            }
            return resultArray;
        }

        double x1 = linePoint1.X - circleCenter.X;
        double y1 = linePoint1.Y - circleCenter.Y;

        double m1 = (linePoint2.Y - linePoint1.Y) / (linePoint2.X - linePoint1.X);
        double quadraticA = 1.0 + Math.pow(m1,2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);
        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(circleRadius,2);
        double quadraticDelta = Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC);

        LinkedList<RobotPoint2D> allPoints = new LinkedList<RobotPoint2D>();
        if(quadraticDelta > 0) {
            double xRoot1 = (-quadraticB + Math.sqrt(quadraticDelta)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.X;
            yRoot1 += circleCenter.Y;
            RobotPoint2D root1 = new RobotPoint2D(xRoot1, yRoot1);
            allPoints.add(root1);

            double xRoot2 = (-quadraticB - Math.sqrt(quadraticDelta)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.X;
            yRoot2 += circleCenter.Y;
            RobotPoint2D root2 = new RobotPoint2D(xRoot2,yRoot2);
            allPoints.add(root2);
        }else if(quadraticDelta == 0){
            double xRoot1 = (-quadraticB) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.X;
            yRoot1 += circleCenter.Y;
            RobotPoint2D root1 = new RobotPoint2D(xRoot1, yRoot1);
            allPoints.add(root1);
        }
        return allPoints;
    }

    public static RobotPoint2D lineCircleIntersection_CLOSEST_TO_POINT_2(RobotPoint2D circleCenter, double circleRadius, RobotPoint2D linePoint1, RobotPoint2D linePoint2){
        LinkedList<RobotPoint2D> allIntersections = lineCircleIntersections(circleCenter,circleRadius,linePoint1,linePoint2);
        int listSize = allIntersections.size();
        if(listSize >= 2){
            if (allIntersections.get(0).distanceTo(linePoint2) > allIntersections.get(1).distanceTo(linePoint2)) {
                return allIntersections.get(1);
            }else{
                return allIntersections.get(0);
            }
        }else if(listSize > 0){
            return allIntersections.get(0);
        }else{
            return null;
        }
    }

    public static RobotPoint2D getNearestPoint(List<RobotPoint2D> allPoints, RobotPoint2D point){
        double smallestDistance = -1;
        RobotPoint2D smallestPoint = null;
        for(RobotPoint2D thisPoint : allPoints){
            double currentDistance = thisPoint.distanceTo(point);
            if(smallestPoint == null || currentDistance < smallestDistance){
                smallestPoint = thisPoint;
                smallestDistance = currentDistance;
            }
        }
        return smallestPoint;
    }

    public static boolean isInBoundingBox(RobotPoint2D point, RobotPoint2D minPoint, RobotPoint2D maxPoint){
        if(point.X >= minPoint.X && point.X <= maxPoint.X && point.Y >= minPoint.Y && point.Y <= maxPoint.Y){
            return true;
        }else{
            return false;
        }
    }

    @Nullable
    public static RobotPoint2D nearestPointOnLineSegment(RobotPoint2D point, RobotPoint2D linePoint1, RobotPoint2D linePoint2){
        if(linePoint1.X == linePoint2.X && linePoint1.Y == linePoint2.Y){
            return linePoint1;
        }
        RobotPoint2D pointResult;
        if(linePoint1.Y == linePoint2.Y){
            pointResult = new RobotPoint2D(point.X,linePoint1.Y);
        }else if(linePoint1.X == linePoint2.X){
            pointResult = new RobotPoint2D(linePoint1.X,point.Y);
        }else{
            double m = (linePoint2.Y - linePoint1.Y) / (linePoint2.X - linePoint1.X);
            double b = linePoint1.Y - linePoint1.X * m;
            double n = -1.0 / m;
            double c = point.Y - point.X * n;
            double nearestPointX = (c-b) / (m-n);
            pointResult = new RobotPoint2D(nearestPointX,m * nearestPointX + b);
        }
        RobotPoint2D boundingBoxMin = new RobotPoint2D(
                Math.min(linePoint1.X,linePoint2.X),
                Math.min(linePoint1.Y,linePoint2.Y)
        );
        RobotPoint2D boundingBoxMax = new RobotPoint2D(
                Math.max(linePoint1.X,linePoint2.X),
                Math.max(linePoint1.Y,linePoint2.Y)
        );
        if(isInBoundingBox(pointResult,boundingBoxMin,boundingBoxMax)){
            return pointResult;
        }else{
            return null;
        }
    }

    @Nullable
    public static RobotPoint2D nearestPointOnLine(RobotPoint2D point, RobotPoint2D linePoint1, RobotPoint2D linePoint2){
        if(linePoint1.X == linePoint2.X && linePoint1.Y == linePoint2.Y){
            return linePoint1;
        }
        RobotPoint2D pointResult;
        if(linePoint1.Y == linePoint2.Y){
            pointResult = new RobotPoint2D(point.X,linePoint1.Y);
        }else if(linePoint1.X == linePoint2.X){
            pointResult = new RobotPoint2D(linePoint1.X,point.Y);
        }else{
            double m = (linePoint2.Y - linePoint1.Y) / (linePoint2.X - linePoint1.X);
            double b = linePoint1.Y - linePoint1.X * m;
            double n = -1.0 / m;
            double c = point.Y - point.X * n;
            double nearestPointX = (c-b) / (m-n);
            pointResult = new RobotPoint2D(nearestPointX,m * nearestPointX + b);
        }
        return pointResult;
    }

    public static RobotPoint2D[] getRobotExtremeBoundingBox(double distMidToFront, double distMidToBack, double distMidToLeft, double distMidToRight){
        RobotPoint2D[] result = new RobotPoint2D[4];
        RobotPoint2D LT = new RobotPoint2D(distMidToFront,distMidToLeft);
        RobotPoint2D RT = new RobotPoint2D(distMidToFront,-distMidToRight);
        RobotPoint2D LB = new RobotPoint2D(-distMidToBack,distMidToLeft);
        RobotPoint2D RB = new RobotPoint2D(-distMidToBack,-distMidToRight);
        result[0] = LT;
        result[1] = RT;
        result[2] = LB;
        result[3] = RB;
        return result;
    }

    public static RobotPose2D fixFieldPosition(RobotPose2D fieldPosition, RobotPoint2D[] robotBondingBox){
        RobotPoint2D[] boundingBoxExtreme = new RobotPoint2D[robotBondingBox.length];
        RobotPose2D fixedFieldPosition = new RobotPose2D(fieldPosition);
        for(int i=0; i<robotBondingBox.length; i++){
            boundingBoxExtreme[i] = getRelativePosition(fieldPosition,robotBondingBox[i]);
        }
        for(int i=0; i<boundingBoxExtreme.length;i++){
            RobotPoint2D currentPoint = boundingBoxExtreme[i];
            double deltaX = 0, deltaY = 0;
            if(currentPoint.X < -SkyStoneCoordinates.FIELD_SIZE_X / 2){
                deltaX = (-SkyStoneCoordinates.FIELD_SIZE_X / 2) - currentPoint.X;
            }else if(currentPoint.X > SkyStoneCoordinates.FIELD_SIZE_X / 2){
                deltaX = -(currentPoint.X - SkyStoneCoordinates.FIELD_SIZE_X / 2);
            }
            if(currentPoint.Y < -SkyStoneCoordinates.FIELD_SIZE_Y / 2){
                deltaY = (-SkyStoneCoordinates.FIELD_SIZE_Y / 2) - currentPoint.Y;
            }else if(currentPoint.Y > SkyStoneCoordinates.FIELD_SIZE_Y / 2){
                deltaY = -(currentPoint.Y - SkyStoneCoordinates.FIELD_SIZE_Y / 2);
            }
            deltaPointArray(boundingBoxExtreme,deltaX,deltaY);
            fixedFieldPosition.X += deltaX;
            fixedFieldPosition.Y += deltaY;
        }
        return fixedFieldPosition;
    }
    private static RobotPoint2D[] deltaPointArray(RobotPoint2D[] array, double x, double y){
        for(int i=0; i<array.length;i++){
            RobotPoint2D currentPoint = array[i];
            currentPoint.X += x;
            currentPoint.Y += y;
        }
        return array;
    }
}