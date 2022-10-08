package org.firstinspires.ftc.teamcode.Reno;

public class FieldFloor
{
    public double destX;
    public double destY;
    public double destZ;
    public FieldTile[][] matrix= new FieldTile[5][5];

    public FieldFloor()
    {
        for (int x = 0; x < matrix.length; x++) {
            for (int y = 0; y < matrix[x].length; y++) {
                matrix[x][y] = new FieldTile(FieldTile.oneTile * (x - 2.5), FieldTile.oneTile * (2.5 - y), 0);

            }
        }
    }

}
