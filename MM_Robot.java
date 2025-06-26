package org.firstinspires.ftc.teamcode;

public class MM_Robot {
    MM_OpMode opMode;
    MM_Drivetrain drivetrain = null;
    MM_Transport transport = null;
    MM_Collectors collectors = null;
    MM_Ascent ascent;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }
    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        transport = new MM_Transport(opMode);
        collectors = new MM_Collectors(opMode);
        ascent = new MM_Ascent(opMode);
    }

}
