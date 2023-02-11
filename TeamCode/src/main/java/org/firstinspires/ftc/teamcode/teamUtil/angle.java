package org.firstinspires.ftc.teamcode.teamUtil;

import static java.lang.Double.NaN;

public class angle {

    public enum angleType{
        CONTINUOUS, ABSOLUTE
    }

    public double value;
    public angleType type;

    public angle(double value, angleType type){
        this.type = type;
        if(type == angleType.ABSOLUTE){
            this.value = value%360;
            if(this.value<0){
                this.value += 360;
            }
        }
        else{
            this.value = value;
        }
    }

    public angle(angleType type) {
        this.type = type;
        this.value = 0;
    }

    public angle(double value){
        this.type = angleType.ABSOLUTE;
        this.value = value%360;
        if(this.value<0){
            this.value += 360;
        }
    }

    /**
     * @param value updates the angle value to the new input. Auto handles overloading if the input is continuous and the angleType is ABSOLUTE.
     */
    public void update(double value){
        if (type == angleType.ABSOLUTE) {
            this.value = value%360;
            if(this.value<0){
                this.value += 360;
            }
        }
        else{
            this.value = value;
        }
    }

    public angle convertToAbsolute(){
        switch (type){
            case ABSOLUTE:
                break;
            case CONTINUOUS:
                this.value = value%360;
                if(this.value<0){
                    this.value += 360;
                }
                this.type = angleType.ABSOLUTE;
                break;
        }
        return this;
    }

    public double angleShortDifference(angle comparator){
        double difference = this.value - comparator.value;
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

    public static angle atanHandler(double planarX, double planarY){
        double atanMeasurement = Math.abs(Math.toDegrees(Math.atan(planarY / planarX)));
        if (planarY >= 0 && planarX >= 0) {
            return new angle(atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY >= 0 && planarX <= 0) {
            return new angle(180 - atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX <= 0) {
            return new angle(180 + atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX >= 0) {
            return new angle(360 - atanMeasurement, angleType.ABSOLUTE);
        }
        else return new angle(NaN, angleType.ABSOLUTE);
    }

    public static angle atanHandler(pose2D startPose2D, pose2D endPose2D){
        double planarX = endPose2D.coordinate2D.x - startPose2D.coordinate2D.x;
        double planarY = endPose2D.coordinate2D.y - startPose2D.coordinate2D.y;
        double atanMeasurement = Math.abs(Math.toDegrees(Math.atan(-planarY / planarX)));
        if (planarY >= 0 && planarX >= 0) {
            return new angle(atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY >= 0 && planarX <= 0) {
            return new angle(180 - atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX <= 0) {
            return new angle(180 + atanMeasurement, angleType.ABSOLUTE);
        }
        else if (planarY <= 0 && planarX >= 0) {
            return new angle(360 - atanMeasurement, angleType.ABSOLUTE);
        }
        else return new angle(NaN, angleType.ABSOLUTE);
    }
}
