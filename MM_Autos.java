package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testAutos", group="test")
public class MM_Autos extends MM_OpMode{
    @Override
    public void runProcedures(){
        multipleTelemetry.addLine("Helloooo");
        multipleTelemetry.update();
    }
}
