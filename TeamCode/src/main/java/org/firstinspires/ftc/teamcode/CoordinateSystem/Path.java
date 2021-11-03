package org.firstinspires.ftc.teamcode.CoordinateSystem;

import org.firstinspires.ftc.teamcode.CoordinateSystem.PathFinder.PathFinder;

import java.util.ArrayList;

public class Path {
    ArrayList<Coordinate> path;

    public Path () {
        path = new ArrayList<Coordinate>();
    }

    public Path (ArrayList<Coordinate> path) {
        this.path = path;
    }

    public ArrayList<Coordinate> getPath() {
        return this.path;
    }

    public void add (Coordinate c) {
        path.add(c);
    }

    public void add (int index, Coordinate c) {
        path.add(index, c);
    }

    public void add (Path p) {
        path.addAll(p.getPath());
    }

    public Coordinate get (int index) {
        return path.get(index);
    }

    public int length () {
        return path.size();
    }

    public ArrayList<Move> getExecutablePath() {
        ArrayList<Move> executablePath = new ArrayList<Move>();
        for (int i = 1; i < path.size(); i++) {
            double slope = (double) (path.get(i).getY() - path.get(i - 1).getY()) / (float) (path.get(i).getX() - path.get(i - 1).getX());
            executablePath.add(new Move(1, slope));
        }
        return executablePath;
    }

    public void goTo(Coordinate place, PathFinder pF) {
        add(pF.getShortestPath(path.get(path.size()-1), place));
    }

    public void goTo(Object place, PathFinder pF) {
        add(pF.getShortestPath(path.get(path.size()-1), place.getClosestCoordinate(path.get(path.size()-1), pF)));
    }
}
