package util;

//Rect class defines a rectagle object
public class Rect {
    //x, y, w, h are the x and y coords of the top left corner and w and h are width and height repectivly
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
    //Gets area of rect
    public int getArea(){
        return w*h;
    }
    //Gets differet corner values where x1 is left x2 right y1 top y2 bottom
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
    //Returns a cropped version of the current rectangle
    public Rect crop(int left, int top, int right, int bottom){
        return new Rect(x+left, y+top,  w-left-right, h-top-bottom);
    }
    //Creates a string representation
    public String toString(){return getX1() + "," + getY1() + "," + getX2() + "," + getY2();}
}
