package org.firstinspires.ftc.teamcode.utils;

public class Field {
    //in
    final static double TILE_SIZE = 24;
    final static double TILE_CONNECTOR_SIZE = 1;
    final static double TILE_SIZE_EDGE = 24.375;
    final static int LANDING_ZONE_WIDTH = 144;
    final static int LANDING_ZONE_LENGTH = 23;
    final static double[] BACKDROP_HEIGHTS = new double[] {12.375,19,25.75,29.75};
    final static double TRUSS_LENGTH_TEAM = 22.5;
    final static double TRUSS_LENGTH_GROUP = 46.25;
    final static double TRUSS_HEIGHT_GROUP_LOW = 23.5;
    final static double TRUSS_HEIGHT_GROUP_HIGH = 26;
    final static double BACKDROP_PARK_LENGTH = 58.375;
    final static double BACKDROP_PARK_WIDTH = 23.125;
    final static double PIXEL_STACK_LENGTH = 11.5;
    final static double PIXEL_OUTSIDE_LEN = 3;
    final static double PIXEL_INSIDE_LEN = 1.25;
    //count
    final static int  PIXEL_ROW = 7;
    final static int PIXEL_COLUMN = 11;
    //April Tags ID
    final static int[] BLUEATID = new int[] {1,2,3};
    final static int[] REDATID = new int[] {4,5,6};
    enum Pixel {
        WHITE,YELLOW,PINK,GREEN
    }
    static Pixel[][] pixels = new Pixel[PIXEL_COLUMN][PIXEL_ROW];


}
