package org.firstinspires.ftc.teamcode;

public class MM_Spline {
    private final double[] xPoints;
    private final double[] yPoints;

    public MM_Spline(double[] xPoints, double[] yPoints, double sections){
        this.xPoints = new double[(int)(sections + 1)];
        this.yPoints = new double[(int)(sections + 1)];
        for (int i = 0; i <= sections; i++){
            double t = (1/sections) * i;
            this.xPoints[i] = xPoints[1] + Math.pow((1-t), 2) * (xPoints[0] - xPoints[1]) + Math.pow(t, 2) * (xPoints[2] - xPoints[1]);
            this.yPoints[i] = yPoints[1] + Math.pow((1-t), 2) * (yPoints[0] - yPoints[1]) + Math.pow(t, 2) * (yPoints[2] - yPoints[1]);
        }
    }

    public MM_Spline(double[] xPoints, double[] yPoints, double sections, boolean isCubic){
        this.xPoints = new double[(int)(sections + 1)];
        this.yPoints = new double[(int)(sections + 1)];
        for (int i = 0; i <= sections; i++){
            double t = (1/sections) * i;

            this.xPoints[i] = Math.pow((1-t), 3)*xPoints[0] + 3*Math.pow((1-t), 2) * t * xPoints[1] + 3*(1-t) * Math.pow(t, 2) * xPoints[2] + Math.pow(t, 3) * xPoints[3];
            this.yPoints[i] = Math.pow((1-t), 3)*yPoints[0] + 3*Math.pow((1-t), 2) * t * yPoints[1] + 3*(1-t) * Math.pow(t, 2) * yPoints[2] + Math.pow(t, 3) * yPoints[3];
        }
    }

    public double[] getNextPoint(int currentSection){
        return new double[]{xPoints[currentSection], yPoints[currentSection]};
    }
}
