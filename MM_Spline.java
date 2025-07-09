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

    public double[] getNextPoint(int currentSection){
        if (currentSection < xPoints.length + 1) {
            return new double[]{xPoints[currentSection], yPoints[currentSection]};
        }
        return new double[]{xPoints[currentSection], yPoints[currentSection]};
    }
}
