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
        if (scoringLocation.equals("Chamber")){
            state = STATES.DRIVE_TO_CHAMBER;
        }
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


                case DRIVE_TO_CHAMBER:
                    MM_Navigation.targetPos.setAll(0, 41.1, 90);
                    MM_Transport.slideTargetInches = 16.1;
                    MM_Transport.targetPivotAngle = 92;
                    if(robot.drivetrain.driveDone() && robot.transport.pivotDone() && robot.transport.slideDone()){
                        state = STATES.SCORE_SPECIMEN;
                    }

                    break;
                case SCORE_SAMPLE:
                    if (previousState != state) {
                        MM_Collectors.score = true;
                    }

                    previousState = state;
                    if (!MM_Collectors.score) {
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
                case SCORE_SPECIMEN:
                    if(state != previousState) {
                        MM_Navigation.targetPos.setHeading(90);
                        MM_Drivetrain.targetDistance = 5.9;
                        MM_Drivetrain.useDistance = true;
                    }

                    if(robot.drivetrain.distanceDriveDone()){
                        MM_Collectors.score = true;
                        state = STATES.COLLECT_SPECIMEN;
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
                        MM_Collectors.collect = true;
                    } else if (previousState != STATES.COLLECT_SAMPLE){
                        MM_Transport.targetPivotAngle = -11; //should be -12 to -14 or so
                        MM_Collectors.collect = true;
                    }

                    previousState = state;
                    if (!MM_Collectors.collect) {
                        state = STATES.DRIVE_TO_BASKET;
                    }
                    break;


                case COLLECT_SPECIMEN:

                    break;
                case DRIVE_TO_PARK:
                    if(!scoringLocation.equals("Chamber")){
                        MM_Navigation.targetPos.setAll(34, 10, 180);
                        MM_Transport.targetPivotAngle = -23.5;
                        MM_Transport.slideTargetInches = .5;
                        MM_Drivetrain.Y_ERROR_THRESHOLD = 6;
                    }
                    if(state == previousState && robot.drivetrain.driveDone()){
                        state = STATES.ASCEND;
                    }
                    previousState = state;

                    break;

                case ASCEND:
                    //ascend with zip tie
                    MM_Navigation.targetPos.setX(28);
                    if(robot.drivetrain.driveDone() && previousState == state){
                        robot.ascent.park();
                    }
                    previousState = state;
            }
            multipleTelemetry.addData("current state", state.name());

            robot.drivetrain.autoRunDrivetrain();
            robot.transport.autoRunPivot();
            robot.transport.autoRunSlide();
            robot.collectors.autoRunCollectorWheels();
            robot.collectors.autoRunSpecClaw();
            telemetry();
        }
    }
}
