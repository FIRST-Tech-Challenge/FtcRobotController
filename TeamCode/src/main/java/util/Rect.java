package util;

public class Rect {
    public int x;
    public int y;
    public int w;
    public int h;
    public Rect(int x1, int y1, int width, int height){
        x = x1;
        y = y1;
        w = width;
        h = height;
    }

    public int getArea(){
        return w*h;
    }
    public int getX1(){
        return x;
    }
    public int getX2(){
        return x+w;
    }
    public int getY1(){
        return y;
    }
    public int getY2(){
        return y+h;
    }

    public Rect crop(int left, int top, int right, int bottom){
        return new Rect(x+left, y+top,  w-left-right, h-top-bottom);
    }

    public String toString(){return getX1() + "," + getY1() + "," + getX2() + "," + getY2();}
}
