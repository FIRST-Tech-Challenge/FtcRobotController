package org.firstinspires.ftc.teamcode.src.robotAttachments.odometry;

import java.util.HashMap;

public enum FieldPoints {
    RedWareHouseInit,
    RedCarouselInit,
    BlueWareHouseInit,
    BlueCarouselInit,
    RedWestLoadingPoint,
    RedEastLoadingPoint,
    RedSouthLoadingPoint,
    RedNorthLoadingPoint,
    BlueEastLoadingPoint,
    BlueWestLoadingPoint,
    BlueSouthLoadingPoint,
    BlueNorthLoadingPoint,
    RedCarouselSpin,
    BlueCarouselSpin,
    RedCarouselPark,
    BlueCarouselPark,
    RedWareHousePark,
    BlueWareHousePark,
    BlueWareHousePass,
    RedWareHousePass,
    RedBarrierPass,
    BlueBarrierPass,
    RedBarrierPark,
    BlueBarrierPark,
    RedSSHA,
    RedSSHB,
    BlueSSHB,
    BlueSSHA;


    public HashMap<FieldPoints, double[]> positionsAndPoints = new HashMap<FieldPoints, double[]>() {{
        put(FieldPoints.RedWestLoadingPoint, new double[]{20, 85});
        put(FieldPoints.RedEastLoadingPoint, new double[]{66, 85.5});
        put(FieldPoints.RedSouthLoadingPoint, new double[]{47, 68});
        put(FieldPoints.RedNorthLoadingPoint, new double[]{47, 100});
        put(FieldPoints.RedCarouselSpin, new double[]{16, 137});
        put(FieldPoints.RedCarouselPark, new double[]{31.2, 130});
        put(FieldPoints.RedWareHousePark, new double[]{8, 34});
        put(FieldPoints.RedWareHousePass, new double[]{8, 58});
        put(FieldPoints.BlueEastLoadingPoint, new double[]{120, 84});
        put(FieldPoints.BlueSouthLoadingPoint, new double[]{94, 68});
        put(FieldPoints.BlueNorthLoadingPoint, new double[]{94, 100});
        put(FieldPoints.BlueWestLoadingPoint, new double[]{80, 85.5});
        put(FieldPoints.BlueCarouselPark, new double[]{102, 130});
        put(FieldPoints.BlueCarouselSpin, new double[]{106, 134});
        put(FieldPoints.BlueWareHousePark, new double[]{137, 34});
        put(FieldPoints.BlueWareHousePass, new double[]{137, 58});
        put(FieldPoints.BlueSSHB, new double[]{107, 7});
        put(FieldPoints.BlueSSHA, new double[]{84, 7});
        put(FieldPoints.RedSSHB, new double[]{34, 7});
        put(FieldPoints.RedSSHA, new double[]{55, 7});
        put(FieldPoints.RedBarrierPass, new double[]{30, 63});
        put(FieldPoints.BlueBarrierPass, new double[]{110, 63});
        put(FieldPoints.RedWareHouseInit, new double[]{7, 63, 90});
        put(FieldPoints.RedCarouselInit, new double[]{7, 101, 90});
        put(FieldPoints.BlueWareHouseInit, new double[]{133, 63, 270});
        put(FieldPoints.BlueCarouselInit, new double[]{133, 101, 270});


    }};

}
