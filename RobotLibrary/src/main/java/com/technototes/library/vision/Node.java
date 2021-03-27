package com.technototes.library.vision;

public class Node {
    public int x,y;
    public int bluer, redr;
    public boolean isup, isdown;
    public Node(int xv, int yv){
        x = xv;
        y = yv;
    }
    public Node setRatings(int r, int b){
        redr = r;
        bluer = b;
        return this;
    }
    public boolean isCloseX(Node n){
        return isClose(n.x, y);
    }
    public boolean isCloseY(Node n){
        return isClose(x, n.y);
    }
    public boolean isClose(int xv, int yv){
        return Math.abs(x-xv) <= 10 && Math.abs(y-yv) <= 10;
    }
    public boolean isClose(Node n){
        return Math.abs(x-n.x) <= 30 && Math.abs(y-n.y) <= 30;
    }

}
