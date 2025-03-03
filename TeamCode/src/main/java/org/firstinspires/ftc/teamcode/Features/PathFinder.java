package org.firstinspires.ftc.teamcode.Features;//import java.util.ArrayList;
//import java.util.List;
//import java.util.Vector;
//
//    public class PathFinder {
//
//        public boolean fullF = false;
//
//        public void setFullF() {
//            fullF = true;
//        }
//
//        public class Island implements Comparable<Island> {
//            public double g, h, x, y; // i = intersections num
//            public int i;
//
//            Island(double g, double h, double x, double y, int i) {
//                this.g = g;
//                this.h = h;
//                this.x = x;
//                this.y = y;
//                this.i = i;
//            }
//
//            @Override
//            public int compareTo(Island other) {
//                return Double.compare(this.getF(), other.getF());
//            }
//
//            public double getF(){
//                if (fullF) return g + h;
//                else return g;
//            }
//
//            public double getG(){
//                return g;
//            }
//
//            public double getH(){
//                return h;
//            }
//
//            public double getX(){
//                return x;
//            }
//
//            public double getY(){
//                return y;
//            }
//
//            public int getI(){
//                return i;
//            }
//        }
//
//        public class Point {
//            public double x, y;
//
//            public Point(double x, double y){
//                this.x = x;
//                this.y = y;
//            }
//        }
//
//        private Island end;
//
//        private final List<Pair<Point, Integer>> rawList;
//        private final List<Island> prossesedList = new ArrayList <>();
//        private final Vector<Island> currentSuccessors = new Vector <>();
//        private final Vector<Point> candidatesList = new Vector <>();
//
//        public PathFinder(List<Pair <Point, Integer>> list) {
//            rawList = list;
//            end = new Island(0.0, 0.0,
//                             list.get(list.size() - 1).first.x,
//                             list.get(list.size() - 1).first.y,
//                             list.get(list.size() - 1).second);
//        }
//
//        private void calculateHeuristics() {
//            double h;
//
//            for (int i = 0; i < rawList.size() - 2; ++i) {
//                h = Math.hypot((end.x - rawList.get(i).first.x), (end.y - rawList.get(i).first.y));
//
//                prossesedList.set(i, new Island(0.0, h, rawList.get(i).first.x,
//                                                rawList.get(i).first.y, rawList.get(i).second));
//            }
//        }
//
//        private void findSuccessors(Island currentIsland) {
//            double g;
//            currentSuccessors.removeAllElements();
//
//            for (int i = 0; i < prossesedList.size() - 1; ++i) {
//                g = Math.hypot((prossesedList.get(i).getX() - currentIsland.x),
//                               (prossesedList.get(i).getY() - currentIsland.y));
//
//                currentSuccessors.add(new Island(g,
//                                                 prossesedList.get(i).getH(),
//                                                 prossesedList.get(i).getX(),
//                                                 prossesedList.get(i).getY(),
//                                                 prossesedList.get(i).getI()
//                ));
//            }
//
//            Collections.sort(currentSuccessors);
//
//            while (currentSuccessors.size() > currentIsland.getI()) {
//                currentSuccessors.remove(currentSuccessors.lastElement());
//            }
//        }
//
//        public Vector navigatePath() {
//            Island currentIsland = prossesedList.get(0);
//            calculateHeuristics();
//
//            while (currentIsland.getH() != 0) {
//                findSuccessors(currentIsland);
//                setFullF();
//                Collections.sort(currentSuccessors);
//                currentIsland = currentSuccessors.get(0);
//                candidatesList.add(new Point(currentIsland.getX(), currentIsland.getY()));
//            }
//
//            return candidatesList;
//        }
//    }
//
//    public static void main (String[] args) {
//        System.out.println("Hello, World.");
//    }
//}
