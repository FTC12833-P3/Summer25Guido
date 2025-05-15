package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autos", group="MM")
public class MM_Autos extends MM_OpMode{
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
        ASCEND
        }
        STATES state = STATES.DRIVE_TO_COLLECT_SAMPLE;
    @Override
    public void runProcedures(){
        while(opModeIsActive()){
            switch (state) {
                case DRIVE_TO_BASKET:
                    state = STATES.COLLECT_SAMPLE;
                    break;
                case SCORE_SAMPLE:
                    MM_Transport.targetPivotAngle = 90;
                    MM_Transport.slideTargetInches = 5;
                    if (robot.transport.slideDone() && robot.transport.pivotDone()){
                        state = STATES.DRIVE_TO_COLLECT_SAMPLE;
                    }
                    break;

                case LOOK_AT_APRILTAG:

                    break;

                case DRIVE_TO_COLLECT_SAMPLE:
                    MM_Transport.targetPivotAngle = 0;
                    MM_Transport.slideTargetInches = 10;
                    MM_Collectors.collect = true;
                    if (robot.transport.slideDone() && robot.transport.pivotDone() &&! MM_Collectors.collect){
                        MM_Collectors.collect = false;
                        state = STATES.SCORE_SAMPLE;
                    }
                    break;

                case COLLECT_SAMPLE:

                    break;

                case DRIVE_TO_PARK:

                    break;

                case ASCEND:

            }

            robot.transport.autoRunPivot();
            robot.transport.autoRunSlide();
            robot.collectors.autoRunCollectorWheels();
        }

    }
}
