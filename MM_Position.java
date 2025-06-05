package org.firstinspires.ftc.teamcode;

public class MM_Position {
    public static double x = 0;
    public static  double y = 0;
    public static double heading = 0;

    public MM_Position(double x, double y, double heading) {
        setAll(x, y, heading);
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getHeading(){
        return heading;
    }

    public void setX(double newX){
        x = newX;
    }

    public void setY(double newY){
        y = newY;
    }

    public void setHeading(double newHeading){
        heading = newHeading;
    }

    public void setAll(double newX, double newY, double newHeading){
        x = newX;
        y = newY;
        heading = newHeading;
    }
}
