package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FixString", group="MM")
public class MM_FixString extends MM_OpMode{
    @Override
    public void runProcedures(){
        while (opModeIsActive()){
            robot.ascent.fixString();

        }
    }
}
