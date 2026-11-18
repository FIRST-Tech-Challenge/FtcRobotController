package org.firstinspires.ftc.teamcode.Utils.Movement;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.MatrixDrawer;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;

public class Pathfinder {

    public static Telemetry telemetry;
    public static float[][] field = new float[141][141];
    public static boolean[][] obstacle = new boolean[141][141];
    public static boolean[][] obstacleDef = new boolean[141][141];
    public static float[][] moveT = {
            {5,4,3,4,5},
            {4,3,2,3,4},
            {3,2,1,2,3},
            {2,1,0.00001f,1,2},
            {3,2,1,2,3},
            {4,3,2,3,4},
            {5,4,3,4,5}
    };
    public static int[][] lastPath;
    public static int[][] pathPoints;
    public static int[] pointAngle;
    public static ArrayList<Position> pos;

    public static void compileDefaultObstaclesLC(int buffer) {
        //Fl1 47 in +y, 6in yW, 5in xW
        //FL2 94 in +x, 6in xW, 5in yW
        //Beam1 51 in +y, 46 in +x, 3in xW, 39in yW
        MatrixDrawer.fillSquare(obstacleDef,0,47-buffer,5+buffer,6+(buffer*2),true);
        MatrixDrawer.fillSquare(obstacleDef,94-buffer,0,6+(buffer*2),5+buffer,true);
        MatrixDrawer.fillSquare(obstacleDef,46-buffer,51-buffer,3+(buffer*2),39+(buffer*2),true);
        MatrixDrawer.fillSquare(obstacleDef,94-buffer,51-buffer,3+(buffer*2),39+(buffer*2),true);
        MatrixDrawer.fillSquare(obstacleDef,135-buffer,93-buffer,5+buffer,6+(buffer*2),true);
        MatrixDrawer.fillSquare(obstacleDef,46-buffer,135-buffer,6+(buffer*2),5+buffer,true);
        MatrixDrawer.fillSquare(obstacleDef,0,0,buffer,141,true);
        MatrixDrawer.fillSquare(obstacleDef,140-buffer,0,buffer,141,true);
        MatrixDrawer.fillSquare(obstacleDef,0,0,141,buffer,true);
        MatrixDrawer.fillSquare(obstacleDef,0,140-buffer,141,buffer,true);
        obstacle = obstacleDef;
    }

    public static void compileCircleMoveT(int r) {
        moveT = new float[(r*2)+1][(r*2)+1];
        for(int x = 0; x < moveT.length; x++) {
            for(int y = 0; y < moveT.length; y++) {
                int x0 = x-r; int y0 = y-r;
                moveT[x][y] = (float) Math.sqrt((x0*x0)+(y0*y0));
            }
        }
    }

    public static void pathFrom(int x, int y) {
        field = new float[141][141];
        field[x][y] = -0.0001f;
        ArrayList<int[]> queue = new ArrayList<>();
        queue.add(new int[] {x,y});
        while(!queue.isEmpty()) {
            for(int x0 = -(moveT.length/2)+1; x0 < (moveT.length/2); x0++) {
                for(int y0 = -(moveT.length/2)+1; y0 < (moveT[0].length/2); y0++) {
                    //System.out.println(x0+(moveT.length/2));
                    int x1 = queue.get(0)[0]; int y1 = queue.get(0)[1];
                    float t = moveT[x0+(moveT.length/2)][y0+(moveT[0].length/2)]+field[x1][y1];
                    if(x1+x0 >= field.length || x1+x0 < 0 || y1+y0 >= field[0].length || y1+y0 < 0) continue;
                    if ((t < field[x1 + x0][y1 + y0] || field[x1 + x0][y1 + y0] == 0) && !obstacle[x1 + x0][y1 + y0]) {
                        field[x1 + x0][y1 + y0] = t;
                        queue.add(new int[]{x1 + x0, y1 + y0});
                    }
                }
            }
            //System.out.println(Arrays.deepToString(queue.toArray()));
            queue.remove(0);
        }
    }

    public static int[][] getPathTo(int x, int y) {
        ArrayList<int[]> path = new ArrayList<>();
        path.add(new int[] {x,y});
        while(field[path.get(path.size()-1)[0]][path.get(path.size()-1)[1]] > 0) {
            float min = -1; int minX = x; int minY = y;
            for(int x0 = -1; x0 <= 1; x0++) {
                for(int y0 = -1; y0 <= 1; y0++) {
                    int x1 = path.get(path.size()-1)[0]; int y1 = path.get(path.size()-1)[1];
                    if(x1+x0 >= field.length || x1+x0 < 0 || y1+y0 >= field[0].length || y1+y0 < 0) continue;
                    float v = field[x1 + x0][y1 + y0];
                    if ((v < min || min == -1) && !obstacle[x1 + x0][y1 + y0]) {
                        min = v;
                        minX = x1 + x0;
                        minY = y1 + y0;
                    }
                }
            }
            path.add(new int[] {minX,minY});
        }
        int[][] out = new int[path.size()][2];
        for(int i = 0; i < out.length; i++) out[i] = path.get(i);
        lastPath = out;
        return out;
    }

    public static int[][] generatePathPoints(int r) {
        ArrayList<int[]> PP = new ArrayList<>();
        int vX = 0;
        int vY = 0;
        for(int i = 0; i < lastPath.length-1; i++) {
            if(lastPath[i+1][0]-lastPath[i][0] != vX || lastPath[i+1][1]-lastPath[i][1] != vY) {
                vX = lastPath[i+1][0]-lastPath[i][0];
                vY = lastPath[i+1][1]-lastPath[i][1];
                PP.add(lastPath[i]);
            }
        }
        PP.add(lastPath[lastPath.length-1]);
        pathPoints = new int[PP.size()][2];
        for(int i = 0; i < PP.size(); i++) {
            pathPoints[i] = PP.get(PP.size()-1-i);
        }
        pointAngle = new int[PP.size()];
        if(pathPoints.length > 2) {
            pointAngle[pointAngle.length - 1] = (int) Math.toDegrees(Math.atan2(
                    pathPoints[pathPoints.length - 1][0] - pathPoints[pathPoints.length - 2][0],
                    pathPoints[pathPoints.length - 1][1] - pathPoints[pathPoints.length - 2][1]
            ));
        }
        pointAngle[0] = r;
        return pathPoints;
    }

    //dist to rotate is rotation/15, dist required to rotate is rotPoly if rot > 45, else 0
    public static void pickRotations() {
        for(int i = 0; i < pathPoints.length-1; i++) {
            float distance = (float) Math.sqrt(((pathPoints[i][0]-pathPoints[i+1][0])*(pathPoints[i][0]-pathPoints[i+1][0]))+((pathPoints[i][1]-pathPoints[i+1][1])*(pathPoints[i][1]-pathPoints[i+1][1])));
            if(i < pathPoints.length-2) {
                System.out.println(rotPoly(getRefAngle((float) (pointAngle[i]-Math.toDegrees(Math.atan2(pathPoints[i+1][0]-pathPoints[i+2][0],pathPoints[i+1][1]-pathPoints[i+2][1])))))-distance);
                float angleDiff = (float) (pointAngle[i]-Math.toDegrees(Math.atan2(pathPoints[i+1][0]-pathPoints[i+2][0],pathPoints[i+1][1]-pathPoints[i+2][1])));
                if(rotPoly(getRefAngle(angleDiff))<distance || angleDiff < 45) {
                    if(Math.abs(angleDiff)%180 < 90) pointAngle[i+1] = (int) Math.toDegrees(Math.atan2(pathPoints[i+1][0]-pathPoints[i+2][0],pathPoints[i+1][1]-pathPoints[i+2][1]));
                    else pointAngle[i+1] = (int) Math.toDegrees(Math.atan2(pathPoints[i+2][0]-pathPoints[i+1][0],pathPoints[i+2][1]-pathPoints[i+1][1]));
                } else {
                    pointAngle[i+1] = pointAngle[i];
                }
            }
        }
    }

    public static float freq = 0.05f;
    public static void generatePoses() {
        pos = new ArrayList<>();
        float time = 0;
        for(int i = 0; i < pathPoints.length-1; i++) {
            Position po0 = new Position(pathPoints[i][0],pathPoints[i][1],pointAngle[i]);
            po0.setTimeStamp((long) (time*1000)); time += freq;
            pos.add(po0);
            float distance = (float) Math.sqrt(((pathPoints[i][0]-pathPoints[i+1][0])*(pathPoints[i][0]-pathPoints[i+1][0]))+((pathPoints[i][1]-pathPoints[i+1][1])*(pathPoints[i][1]-pathPoints[i+1][1])));
            float angle = (float) (Math.atan2(pathPoints[i+1][0]-pathPoints[i][0],pathPoints[i+1][1]-pathPoints[i][1]));
            float sin = (float) Math.sin(angle); float cos = (float) Math.cos(angle);
            float d = 0;
            float a = 0;
            ArrayList<Position> po2 = new ArrayList<>();
            while(d+(MovementUtils.velocity(0,a)*freq)+1 < distance/2.0f) {
                d = MovementUtils.distanceCovered(0,a);
                Position p = new Position(
                        pathPoints[i][0]+(sin*d),
                        pathPoints[i][1]+(cos*d),
                        0);
                p.setTimeStamp((long) (time*1000));
                pos.add(p);
                po2.add(new Position(
                        pathPoints[i+1][0]-(sin*d),
                        pathPoints[i+1][1]-(cos*d),
                        0));
                a += freq;
                time += freq;
            }
            for(int b = po2.size()-1; b >= 0; b--) {
                po2.get(b).setTimeStamp((long) (time*1000));
                pos.add(po2.get(b));
                time += freq;
            }
        }
        pos.add(new Position(
                pathPoints[pathPoints.length-1][0],
                pathPoints[pathPoints.length-1][1],
                pointAngle[pathPoints.length-1]));
    }

    public static void pathfind(int x0, int y0, int r, int x1, int y1) {
        pathFrom(x0,y0);
        getPathTo(x1,y1);
        generatePathPoints(r);
        pickRotations();
        generatePoses();
    }

    public static float rotPoly(float r) {
        return (-0.0003f*r*r*r)+(0.0711f*r*r)-(4.3889f*r)+93;
    }

    public static float getRefAngle(float r) {
        return (float) (180*Math.abs((r/180.0f)-Math.floor((r/180.0f)+0.5)));
    }

    public static void saveI(int scale, int quality, String name) {
        Bitmap bit = Bitmap.createBitmap(141*scale,141*scale, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(bit);
        for(int x = 0; x < field.length; x++) {
            for(int y = 0; y < field[0].length; y++) {
                float b = 1;
                if(lastPath != null) {
                    for(int[] p : lastPath) {
                        if(p[0] == x && p[1] == y) {
                            b = 0.5f;
                            break;
                        }
                    }
                }
                if(pathPoints != null) {
                    for(int[] p : pathPoints) {
                        if(p[0] == x && p[1] == y) {
                            b = 0.1f;
                            break;
                        }
                    }
                }
                Paint p = new Paint();
                p.setColor(Color.HSVToColor(new float[] {field[x][y]/1f,1,b})); //1f for field gradient, 0.25f for ripple
                if(obstacle[x][y]) p.setColor(Color.BLACK);
                c.drawRect(x*scale,y*scale,x*scale+scale,y*scale+scale,p);
            }
        }
        try {
            File f = new File(MiscUtils.dataFolder+"/"+name+".jpeg");
            f.createNewFile();
            FileOutputStream fO = new FileOutputStream(f);
            bit.compress(Bitmap.CompressFormat.JPEG,quality,fO);
            fO.flush();
        } catch (Exception e) {
            if(telemetry != null) {
                telemetry.addData("Error", e.toString());
                StackTraceElement[] St = e.getStackTrace();
                for (int i = 0; i < St.length; i++) telemetry.addLine(St[i].toString());
            }
        }
    }

    public static void saveTxT(String name) {
        StringBuilder s = new StringBuilder();
        for(int i = 0; i < pathPoints.length; i++) {
            s.append("X: ");
            s.append(pathPoints[i][0]);
            s.append(", Y: ");
            s.append(pathPoints[i][1]);
            s.append(", R: ");
            s.append(pointAngle[i]);
            if(i < pathPoints.length-1) s.append("; ");
        }
        try {
            File f = new File(MiscUtils.dataFolder + "/" + name + ".txt");
            f.createNewFile();
            FileOutputStream fO = new FileOutputStream(f);
            fO.write(s.toString().getBytes());
            fO.flush();
        } catch (Exception e) {
            if(telemetry != null) {
                telemetry.addData("Error", e.toString());
                StackTraceElement[] St = e.getStackTrace();
                for (int i = 0; i < St.length; i++) telemetry.addLine(St[i].toString());
            }
        }
    }

}
