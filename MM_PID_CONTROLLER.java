package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MM_PID_CONTROLLER {
    private final ElapsedTime loopTime = new ElapsedTime();

    public static double P_COEFF = 0.02083;
    //public static double I_COEFF;
    public static double D_COEFF = 0;

    private double prevError = 0;

    public MM_PID_CONTROLLER(double P_CO_EFF, double I_CO_EFF, double D_CO_EFF){
        P_COEFF = P_CO_EFF;
        //I_COEFF = I_CO_EFF;
        D_COEFF = D_CO_EFF;
    }

    public double getPID(double error){
        double P = error * P_COEFF;
        double I = 0; //no I for now
        double D = (error - prevError) / loopTime.milliseconds() * D_COEFF;
        loopTime.reset();

        prevError = error;
        return P + I + D;
    }
}
