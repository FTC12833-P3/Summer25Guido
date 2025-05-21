package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_PID_CONTROLLER {
    private final ElapsedTime loopTime = new ElapsedTime();

    private final double P_CO_EFF;
    //private final double I_CO_EFF;
    private final double D_CO_EFF;

    private double prevError = 0;

    public MM_PID_CONTROLLER(double P_CO_EFF, double I_CO_EFF, double D_CO_EFF){
        this.P_CO_EFF = P_CO_EFF;
        //this.I_CO_EFF = I_CO_EFF;
        this.D_CO_EFF = D_CO_EFF;
    }

    public double getPID(double error){
        double P = error * P_CO_EFF;
        double I = 0; //no I for now
        double D = (error - prevError) / loopTime.milliseconds() * D_CO_EFF;
        loopTime.reset();

        return P + I + D;
    }
}
