package org.firstinspires.ftc.teamcode;

public class MM_Spline {
    private final double[] xPoints;
    private final double[] yPoints;
    private final double[] sectionLengths;
    private double fullCurveLength = 0;
    private double distanceTraveled = 0;

    public MM_Spline(double[] xPoints, double[] yPoints, int sections){
        this.xPoints = new double[(int)(sections + 1)];
        this.yPoints = new double[(int)(sections + 1)];
        sectionLengths = new double[sections + 1];
        for (int i = 0; i <= sections; i++){
            double t = (1.0 /sections) * i;
            this.xPoints[i] = xPoints[1] + Math.pow((1-t), 2) * (xPoints[0] - xPoints[1]) + Math.pow(t, 2) * (xPoints[2] - xPoints[1]);
            this.yPoints[i] = yPoints[1] + Math.pow((1-t), 2) * (yPoints[0] - yPoints[1]) + Math.pow(t, 2) * (yPoints[2] - yPoints[1]);
            if (i >0){
                sectionLengths[i - 1] = Math.hypot(this.xPoints[i-1] - this.xPoints[i], this.yPoints[i-1] - this.yPoints[i]);
                fullCurveLength += sectionLengths[i - 1];
            }
        }
        sectionLengths[sections] = Math.hypot(this.xPoints[sections -1] - this.xPoints[sections], this.yPoints[sections-1] - this.yPoints[sections]);
        fullCurveLength += sectionLengths[sections];
    }

    public MM_Spline(double[] xPoints, double[] yPoints, int sections, boolean isCubic) {
        this.xPoints = new double[(int) (sections + 1)];
        this.yPoints = new double[(int) (sections + 1)];
        sectionLengths = new double[sections + 1];
        int splinePointIncrement = 0;
        for (int c = 0; c <= xPoints.length / 4; c += (sections + 1)){
            for (int i = 0; i <= sections; i++) {
                double t = (1.0 / sections) * i;

                this.xPoints[i + c] = Math.pow((1 - t), 3) * xPoints[0 + splinePointIncrement] + 3 * Math.pow((1 - t), 2) * t * xPoints[1 + splinePointIncrement] + 3 * (1 - t) * Math.pow(t, 2) * xPoints[2 + splinePointIncrement] + Math.pow(t, 3) * xPoints[3 + splinePointIncrement];
                this.yPoints[i + c] = Math.pow((1 - t), 3) * yPoints[0 + splinePointIncrement] + 3 * Math.pow((1 - t), 2) * t * yPoints[1 + splinePointIncrement] + 3 * (1 - t) * Math.pow(t, 2) * yPoints[2 + splinePointIncrement] + Math.pow(t, 3) * yPoints[3 + splinePointIncrement];
                if (i > 0) {
                    sectionLengths[(i + c) - 1] = Math.hypot(this.xPoints[(i + c) - 1] - this.xPoints[(i + c)], this.yPoints[(i + c) - 1] - this.yPoints[(i + c)]);
                }
            }
            splinePointIncrement += 4;
            sectionLengths[sections + c] = Math.hypot(this.xPoints[(sections + c) - 1] - this.xPoints[sections + c], this.yPoints[(sections + c) - 1] - this.yPoints[sections + c]);
        }
    }

    public double[] getNextPoint(int currentSection){
        return new double[]{xPoints[currentSection], yPoints[currentSection]};
    }

    public void updateDistanceTraveled(int currentSection){
        distanceTraveled += sectionLengths[currentSection];
    }

    public double getError(){
        return fullCurveLength - distanceTraveled;
    }

    public void resetDistanceTraveled(){
        distanceTraveled = 0;
    }

    public boolean splineDone(int currentSection) {
        return true;
    }
}
