package org.firstinspires.ftc.teamcode.ebotsenums;

public enum RobotSize {
    xSize(CsysDirection.X, 18.0),
    ySize(CsysDirection.Y, 13.0),
    zSize(CsysDirection.Z, 18.0);

    CsysDirection csysDirection;
    double sizeValue;
    final static double bucketOffset = 1.0;

    RobotSize(CsysDirection csysDirectionIn, double sizeValueIn){
        this.csysDirection = csysDirectionIn;
        this.sizeValue = sizeValueIn;
    }

    public CsysDirection getCsysDirection() {
        return csysDirection;
    }

    public double getSizeValue() {
        return sizeValue;
    }

    public static double getBucketOffset() {
        return bucketOffset;
    }
}
