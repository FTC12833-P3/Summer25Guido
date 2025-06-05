package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autos", group="MM")
public class MM_Autos extends MM_OpMode{
    private int cycles = 0;

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
    STATES state = STATES.DRIVE_TO_BASKET;
    STATES previousState = null;

    @Override
    public void runProcedures(){
        while(opModeIsActive()){
            switch (state) {
                case DRIVE_TO_BASKET:
                    //drive (x, y, heading)
                    MM_Navigation.targetPos.setAll(53.8, 52.5, 225);
                    MM_Transport.targetPivotAngle = 93;
                    MM_Transport.slideTargetInches = 26;
                    previousState = state;
                    if (robot.drivetrain.driveDone() && robot.transport.slideDone() && robot.transport.pivotDone()) {
                        state = STATES.SCORE_SAMPLE;
                    }
                    break;
                case SCORE_SAMPLE:
                    if (previousState != state) {
                        MM_Collectors.wheelsScore = true;
                    }

                    previousState = state;
                    if (!MM_Collectors.wheelsScore) {
                        cycles ++;

//                        if (scoringLocation.equals("basket") && cycles > 4) {
//                            state = STATES.ASCEND;
//                        } else if (scoringLocation.equals("chamber") && cycles > 4) {
//                            state = STATES.DRIVE_TO_PARK;
//                        } else {

                            state = STATES.DRIVE_TO_COLLECT_SAMPLE;

                        //}
                    }
                    break;

                case LOOK_AT_APRILTAG:
                    //drive (x, y, heading)
                    //AprilTag stuff

                    previousState = state;
                    state = STATES.DRIVE_TO_COLLECT_SAMPLE;
                    break;

                case DRIVE_TO_COLLECT_SAMPLE:
                    //drive (x, y, heading)
                    if(cycles < 3) {
                        MM_Navigation.targetPos.setAll(46.6 + (10 * (cycles - 1)), 38.5, 270);
                        MM_Transport.slideTargetInches = 5.5;
                        MM_Transport.targetPivotAngle = 0;
                    } else if (cycles < 4){
                        MM_Navigation.targetPos.setAll(58.75, 46.8, 285.86);
                        MM_Transport.targetPivotAngle = 0;
                        MM_Transport.slideTargetInches = 15.5;
                    }


                    previousState = state;
                    if (robot.drivetrain.driveDone() && robot.transport.slideDone() && robot.transport.pivotDone()) {
                        if(cycles < 4) {
                            state = STATES.COLLECT_SAMPLE;
                        } else {
                            state = STATES.DRIVE_TO_PARK;
                        }
                    }
                    break;

                case COLLECT_SAMPLE:
                    if (previousState != STATES.COLLECT_SAMPLE && cycles < 3) {
                        MM_Transport.targetPivotAngle = -14.2; //should be -12 to -14 or so
                        MM_Collectors.wheelsCollect = true;
                    } else if (previousState != STATES.COLLECT_SAMPLE){
                        MM_Transport.targetPivotAngle = -11; //should be -12 to -14 or so
                        MM_Collectors.wheelsCollect = true;
                    }

                    previousState = state;
                    if (!MM_Collectors.wheelsCollect) {
                        state = STATES.DRIVE_TO_BASKET;
                    }
                    break;

                case DRIVE_TO_PARK:
                    if(!scoringLocation.equals("Chamber")){
                        MM_Navigation.targetPos.setAll(33, 10, 180);
                        MM_Transport.targetPivotAngle = -23.5;
                        MM_Transport.slideTargetInches = .5;
                        MM_Drivetrain.X_ERROR_THRESHOLD = .2;
                        MM_Drivetrain.Y_ERROR_THRESHOLD = .2;
                    }
                    if(robot.drivetrain.driveDone() && state == previousState){
                        state = STATES.ASCEND;
                    }
                    previousState = state;

                    break;

                case ASCEND:
                    //ascend with zip tie
                    if(previousState == state && robot.drivetrain.driveDone()){
                        robot.ascent.park();
                    }
                    previousState = state;
            }
            multipleTelemetry.addData("current state", state.name());

            robot.drivetrain.autoRunDrivetrain();
            robot.transport.autoRunPivot();
            robot.transport.autoRunSlide();
            robot.collectors.autoRunCollectorWheels();
            telemetry();
        }
    }
}
