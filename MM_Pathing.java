package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="pathing", group="MM")
public class MM_Pathing extends MM_OpMode{

    private enum STATES {
        DRIVE_TO_COLLECT_SAMPLE,
        COLLECT_SAMPLE,
        DRIVE_TO_BASKET,
        SCORE_SAMPLE,
        DONE_SCORE_SAMPLE,
        DRIVE_TO_COLLECT_SPECIMEN,
        COLLECT_SPECIMEN,
        DRIVE_TO_CHAMBER,
        SCORE_SPECIMEN,
        LOOK_AT_APRILTAG,
        DRIVE_TO_PARK,
        ASCEND,
        PATHING
    }
    STATES state = STATES.DRIVE_TO_BASKET;
    STATES previousState = null;

    int currentSection = 0;
    double targetX;
    double targetY;

    @Override
    public void runProcedures(){
        if (scoringLocation.equals("Chamber")){
            state = STATES.DRIVE_TO_CHAMBER;
        }

        prepareToSpline(robot.drivetrain.navigation.testCubicSpline);
        while(opModeIsActive()){

            if(robot.drivetrain.driveDone()){
                setNextSplinePoint(robot.drivetrain.navigation.testCubicSpline);
                multipleTelemetry.addData("xPoint", targetX);
                multipleTelemetry.addData("yPoint", targetY);
                multipleTelemetry.update();
            }

            if(splineDone()){
                //state = ...
            }

            robot.drivetrain.autoRunDrivetrain();
        }
    }


    public void setNextSplinePoint(MM_Spline spline){
        targetX = spline.getNextPoint(currentSection)[0];
        targetY = spline.getNextPoint(currentSection)[1];
        MM_Position_Data.targetPos.setAll(targetX, targetY, -90);
        currentSection++;
    }

    public void prepareToSpline(MM_Spline spline){
        setNextSplinePoint(spline);
        MM_Drivetrain.X_ERROR_THRESHOLD = 4;
        MM_Drivetrain.Y_ERROR_THRESHOLD = 4;
        MM_PID_CONTROLLER.D_COEFF = 0;
        currentSection -= 1;
    }
    public boolean splineDone(){
        if(currentSection == MM_Autos.SPLINE_DETAIL_LEVEL + 1){
            MM_Drivetrain.X_ERROR_THRESHOLD = .5;
            MM_Drivetrain.Y_ERROR_THRESHOLD = .5;
            MM_PID_CONTROLLER.D_COEFF = 30;
            return true;
        }
        return false;
    }
}
