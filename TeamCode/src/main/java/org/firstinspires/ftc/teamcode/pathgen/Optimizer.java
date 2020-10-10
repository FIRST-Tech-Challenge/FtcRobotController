//package org.firstinspires.ftc.teamcode.pathgen;
//
//public class Optimizer {
//    public static void main(String[] args) {
//        display display = new display();
//
//        Path path = Path.ofPoints(0, Math.PI / 2,
//                new PathPoint(50, 50),
//                new PathPoint(200, 200),
//                new PathPoint(30, 250),
//                new PathPoint(330, 250),
//                new PathPoint(290, 400));
//
//        display.show(path);
//        path.subdivide(false);
//        path.subdivide(false);
//
//        while (!display.exited) {
//            path.velocities();
//            path.move(1);
//
//            if (display.subdivideReq) {
//                path.subdivide(display.tightTurnsOnlyC);
//                display.subdivideReq = false;
//            }if (display.addControlReq) {
//                var insert = new PathPoint(450, 450);
//                insert.isControl = true;
//                path.insert(insert);
//
//                display.addControlReq = false;
//                path.updateEnds();
//            }
//        }
//        ExportPath.write(path);
//        System.exit(0);
//    }
//}