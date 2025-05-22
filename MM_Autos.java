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
                    MM_Drivetrain.desiredPower = 0.25;
                    MM_Navigation.targetPos.setX(24);
                    MM_Transport.targetPivotAngle = 90;
                    MM_Transport.slideTargetInches = 15; //52

                    previousState = state;
                    if (robot.transport.slideDone() && robot.transport.pivotDone() && robot.drivetrain.driveDone()) {
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

                        if (scoringLocation.equals("basket") && cycles > 4) {
                            state = STATES.ASCEND;
                        } else if (scoringLocation.equals("chamber") && cycles > 4) {
                            state = STATES.DRIVE_TO_PARK;
                        } else {
                            state = STATES.LOOK_AT_APRILTAG;
                        }
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
                    MM_Transport.slideTargetInches = 5.5;
                    MM_Transport.targetPivotAngle = 0;

                    previousState = state;
                    if (robot.transport.slideDone() && robot.transport.pivotDone()) {
                        state = STATES.COLLECT_SAMPLE;
                    }
                    break;

                case COLLECT_SAMPLE:
                    if (previousState != STATES.COLLECT_SAMPLE) {
                        MM_Transport.targetPivotAngle = 0; //should be -12 to -14 or so
                        MM_Collectors.wheelsCollect = true;
                    }

                    previousState = state;
                    if (!MM_Collectors.wheelsCollect) {
                        state = STATES.DRIVE_TO_BASKET;
                    }
                    break;

                case DRIVE_TO_PARK:
                    //drive (x, y, heading)

                    previousState = state;
                    break;

                case ASCEND:
                    //drive (x, y, heading)
                    //ascend with zip tie
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
