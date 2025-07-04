package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Config
public class MM_Navigation {
    public static final double TAG_FLIP_THRESHOLD = .2;
    private final MM_OpMode opMode;

    public MM_VisionPortal visionPortal;
    public GoBildaPinpointDriver odometryController;

    Pose2D currentPos;
    Pose2D AprilTagPos;
    double pastExtrinsicY;
    public static MM_Position targetPos = new MM_Position(0, 0, 0);
    public MM_Spline testSpline = new MM_Spline(new double[]{36, 40.32, 43.68, 46, 47.52, 48, 47.52, 46.08, 43.68, 40.32, 36}, new double[]{-24, -19.2, -14.4, -9.6, -4.8, 0, 4.8, 9.6, 14.4, 19.2, 24});

    MM_Navigation(MM_OpMode opMode){
        this.opMode = opMode;
        visionPortal = new MM_VisionPortal(opMode);
        odometryController = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometryController.setOffsets(53.975, 3.175);
        odometryController.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryController.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if(opMode.getClass() == MM_Autos.class) {
            odometryController.resetPosAndIMU();
        }

        odometryController.update();
        currentPos = odometryController.getPosition();
        targetPos.setAll(0, 0, 0);
    }

    public void updatePosition(){
        updatePosition(false);
    }

    public void updatePosition(boolean useApriltag){
        currentPos = odometryController.getUpdatedPositon();

        opMode.multipleTelemetry.addData("xOdom", round2Dec(getX()));
        opMode.multipleTelemetry.addData("yOdom", round2Dec(getY()));
        opMode.multipleTelemetry.addData("yawOdom", round2Dec(getHeading()));

        if (useApriltag) {
            if (AprilTagPos != null) {
                pastExtrinsicY = AprilTagPos.getY(DistanceUnit.INCH);
            }
            AprilTagPos = visionPortal.setPosFromApriltag();
            if (AprilTagPos != null) {
                opMode.multipleTelemetry.addData("xApril", round2Dec(AprilTagPos.getX(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yApril", round2Dec(AprilTagPos.getY(DistanceUnit.INCH)));
                opMode.multipleTelemetry.addData("yawApril", round2Dec(AprilTagPos.getHeading(AngleUnit.DEGREES)));

                if ((opMode.opModeInInit() && MM_OpMode.currentGamepad1.b && !MM_OpMode.previousGamepad1.b) || (!opMode.opModeInInit() && opMode.getClass() != MM_Autos.class)) {
                    odometryController.setPosition(AprilTagPos);
                    if(opMode.opModeInInit()){
                        MM_OpMode.alliance = MM_VisionPortal.startingTag == 13 || MM_VisionPortal.startingTag == 11? -1: 1;
                        MM_OpMode.scoringLocation = MM_VisionPortal.startingTag == 13 || MM_VisionPortal.startingTag == 16? "Basket": "Chamber";
//                        if(MM_OpMode.goal.equals("Basket")){ Only for spec claw
//                            opMode.robot.collector.specClaw.setPosition(MM_CONSTANTS.COLLECT_CONSTANTS.SPEC_OPEN_POS);
//                        }
                    }
                }
            } else { //just here for dashboard
                opMode.multipleTelemetry.addData("xApril", "");
                opMode.multipleTelemetry.addData("yApril", "");
                opMode.multipleTelemetry.addData("yawApril", "");
            }
        }

        currentPos = odometryController.getUpdatedPositon();
    }

    private double round2Dec(double inDouble) {
        return Math.round(inDouble * 100) / 100.0;
    }

    public double getX() {
        return currentPos.getX(DistanceUnit.INCH); }


    public double getY() {
        return currentPos.getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return currentPos.getHeading(AngleUnit.DEGREES);
    }

    public void setPosition(double xPos, double yPos, double yawPos){
        odometryController.setPosition(new Pose2D(DistanceUnit.INCH, xPos, yPos, AngleUnit.DEGREES, yawPos));
    }
}