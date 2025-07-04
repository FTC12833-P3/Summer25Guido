package org.firstinspires.ftc.teamcode;

public class MM_Spline {
    private final double[] xPoints;
    private final double[] yPoints;



    public MM_Spline(double[] xPoints, double[] yPoints){
        this.xPoints = xPoints;
        this.yPoints = yPoints;
    }

    public double[] getNextPoint(int currentSection){
        return new double[]{xPoints[currentSection + 1], yPoints[currentSection + 1]};
    }
}
