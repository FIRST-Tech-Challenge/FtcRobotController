package org.firstinspires.ftc.teamcode.pathgen;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class ExportPath {
    public static void write(Path path){
        StringBuilder text = new StringBuilder();
        for(PathPoint p:path){
            text.append(p.x).append(" ")
                    .append(p.y).append(" ")
                    .append(p.speed).append(" ")
                    .append(p.dir)
                    .append("\n");
        }
        try {
            Files.write(Paths.get("./paths.txt"), text.toString().getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
