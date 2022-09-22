package org.firstinspires.ftc.teamcode.ebotsutil;

import org.firstinspires.ftc.teamcode.ebotsenums.CoordinateSystem;
import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;

import java.util.Formatter;

public class FieldPosition {
    private double yPosition;
    private double xPosition;
    private double zPosition;
    private CoordinateSystem coordinateSystem;  //Default csys is FIELD

    public FieldPosition(){
        this.xPosition = 0.0;
        this.yPosition = 0.0;
        this.zPosition = 0.0;
        this.coordinateSystem = CoordinateSystem.FIELD;
    }
    public FieldPosition(double x,double y){
        this.xPosition = x;
        this.yPosition = y;
        this.zPosition = 0.0;
        this.coordinateSystem = CoordinateSystem.FIELD;

    }

    public FieldPosition(double x,double y, CoordinateSystem csys){
        this.xPosition = x;
        this.yPosition = y;
        this.zPosition = 0.0;
        this.coordinateSystem = csys;

    }

    public FieldPosition(double x,double y, double z){
        this(x,y);
        this.zPosition = z;
    }

    public FieldPosition(double x,double y, double z, CoordinateSystem csys){
        this(x,y);
        this.zPosition = z;
        this.coordinateSystem = csys;
    }

    /**************************************************
     * GETTERS
     *************************************************/
    public double getyPosition() {
        return yPosition;
    }

    public double getxPosition() {
        return xPosition;
    }

    public double getzPosition() {
        return zPosition;
    }

    public double getPositionComponent(CsysDirection dir){
        double returnValue = 0;
        if(dir == CsysDirection.X) returnValue = xPosition;
        else if(dir == CsysDirection.Y) returnValue = yPosition;
        else if(dir == CsysDirection.Z) returnValue = zPosition;
        return returnValue;
    }

    public CoordinateSystem getCoordinateSystem(){return coordinateSystem;}

    /**************************************************
     * SETTERS
     *************************************************/
    public void setyPosition(double yPosition) {
        this.yPosition = yPosition;
    }

    public void setxPosition(double xPosition) {
        this.xPosition = xPosition;
    }

    public void setzPosition(double zPosition) {
        this.zPosition = zPosition;
    }

    /**************************************************
     * Instance Methods
     *************************************************/
    public double getXYMagnitude(){
        return Math.hypot(xPosition, yPosition);
    }

    public double getFieldErrorDirectionDeg(){
        return Math.toDegrees(Math.atan2(yPosition, xPosition));
    }

    public void offsetInPlace(FieldPosition offset){
        this.xPosition += offset.xPosition;
        this.yPosition += offset.yPosition;
    }

    public FieldPosition offsetFunc(FieldPosition offset){
        FieldPosition newFieldPosition = new FieldPosition(this.xPosition, this.yPosition);
        newFieldPosition.xPosition += offset.xPosition;
        newFieldPosition.yPosition += offset.yPosition;
        return newFieldPosition;
    }

    @Override
    public String toString(){
        StringBuilder sb = new StringBuilder();
        Formatter fmt = new Formatter(sb);
        sb.append("Field Position: (");
        fmt.format("%.2f", xPosition);
        sb.append(", ");
        fmt.format("%.2f", yPosition);
        sb.append(", ");
        fmt.format("%.2f", zPosition);
        sb.append(")");

        return sb.toString();
    }

}
