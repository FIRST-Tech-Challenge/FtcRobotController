package org.firstinspires.ftc.teamcode.pathgen;

import java.io.*;
import java.util.StringTokenizer;

public class ImportPath {
    public static Path getPath (String path){
        Path ret = new Path();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                StringTokenizer t = new StringTokenizer(line, ",");
                double x = Integer.parseInt(t.nextToken());
                double y = Integer.parseInt(t.nextToken());
                double speed = Integer.parseInt(t.nextToken());
                double dir = Integer.parseInt(t.nextToken());

                PathPoint toAdd = new PathPoint(x, y);
                toAdd.speed = speed;
                toAdd.dir = dir;

                ret.add(toAdd);
                return ret;
            }
        }catch (IOException e) {
            System.out.printf("File %s not found%n", path);
        }

        return null;
    }
}
