package org.firstinspires.ftc.teamcode.teamUtil;

import static java.lang.Double.NaN;

public class Angle {
    
    public double getValue() {
        return value;
    }
    
    public enum angleType{
        CONTINUOUS, ABSOLUTE
    }

    private double value;
    public angleType type;

    public Angle(double value, angleType type){
        this.type = type;
        if(type == angleType.ABSOLUTE){
            this.value = value%360;
            if(this.getValue() <0){
                this.value = this.getValue() + 360;
            }
        }
        else{
            this.value = value;
        }
    }

    public Angle(angleType type) {
        this.type = type;
        this.value = 0;
    }

    public Angle(double value){
        this.type = angleType.ABSOLUTE;
        this.value = value%360;
        if(this.value <0){
            this.value += 360;
        }
    }

    /**
     * @param value updates the angle value to the new input. Auto handles overloading if the input is continuous and the angleType is ABSOLUTE.
     */
    public void update(double value){
        if (type == angleType.ABSOLUTE) {
            this.value = value%360;
            if(this.getValue() <0){
                this.value = this.getValue() + 360;
            }
        }
        else{
            this.value = value;
        }
    }

    public Angle convertToAbsolute(){
        switch (type){
            case ABSOLUTE:
                break;
            case CONTINUOUS:
                this.value = getValue() %360;
                if(this.getValue() <0){
                    this.value = this.getValue() + 360;
                }
                this.type = angleType.ABSOLUTE;
                break;
        }
        return this;
    }

    public double angleShortDifference(Angle comparator){
        double difference = this.getValue() - comparator.getValue();
        if (difference > 180) {
            return (-360) + difference;
        }
        else if (difference < -180) {
            return 360 + difference;
        }
        else {
            return difference;
        }
    }

    public static Angle atanHandler(double x, double y){
        double atanMeasurement = Math.abs(Math.toDegrees(Math.atan(y / x)));
        if (y >= 0 && x >= 0) {
            return new Angle(atanMeasurement, angleType.ABSOLUTE);
        }
        else if (y >= 0 && x <= 0) {
            return new Angle(180 - atanMeasurement, angleType.ABSOLUTE);
        }
        else if (y <= 0 && x <= 0) {
            return new Angle(180 + atanMeasurement, angleType.ABSOLUTE);
        }
        else if (y <= 0 && x >= 0) {
            return new Angle(360 - atanMeasurement, angleType.ABSOLUTE);
        }
        else return new Angle(NaN, angleType.ABSOLUTE);
    }

    public static Angle atanHandler(Pose2D startPose2D, Pose2D endPose2D){
        double planarX = endPose2D.coordinate2D.x - startPose2D.coordinate2D.x;
        double planarY = endPose2D.coordinate2D.y - startPose2D.coordinate2D.y;
        double atanMeasurement = Math.abs(Math.toDegrees(Math.atan(-planarY / planarX)));
        if (planarY >= 0 && planarX >= 0) {
            return new Angle(atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY >= 0 && planarX <= 0) {
            return new Angle(180 - atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX <= 0) {
            return new Angle(180 + atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX >= 0) {
            return new Angle(360 - atanMeasurement, angleType.ABSOLUTE);
        }
        else return new Angle(NaN, angleType.ABSOLUTE);
    }
}
