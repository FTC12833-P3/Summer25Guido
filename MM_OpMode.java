package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class MM_OpMode extends LinearOpMode {
    MM_Robot robot = null;

    public static Gamepad currentGamepad1 = new Gamepad();
    public static Gamepad previousGamepad1 = new Gamepad();

    public static Gamepad currentGamepad2 = new Gamepad();
    public static Gamepad previousGamepad2 = new Gamepad();

    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    public static String scoringLocation = "Basket";
    public static int alliance = -1;
    public MM_Spline currentSpline = null;

    public void runOpMode(){
        multipleTelemetry.addData("Status", "Initializing... please wait");
        multipleTelemetry.update();
        initialize();
        while (opModeInInit()){
            multipleTelemetry.addData("Status", "Initialized");
            multipleTelemetry.update();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            if (getClass() == MM_Autos.class ||getClass() == MM_Pathing.class){
                robot.drivetrain.navigation.updatePosition(true);
            }
        }
        runProcedures();
        if(isStopRequested()){
            MM_Position_Data.targetPos.setAll(0,0,0);
            MM_Transport.slideTargetInches = 0;
            MM_Transport.targetPivotAngle = 0;
            MM_Collectors.collect = false;
            MM_Collectors.score = false;
            MM_Drivetrain.Y_ERROR_THRESHOLD = .5;
            MM_Drivetrain.X_ERROR_THRESHOLD = .5;
            MM_Drivetrain.desiredPower = 1;
        }
    }

    public abstract void runProcedures();

    public void telemetry(){
        multipleTelemetry.addData("slide", "inches: %.2f ticks: %d", robot.transport.getSlideInches(), robot.transport.getSlideTicks());
        multipleTelemetry.addData("pivot", "degrees: %.2f targetAngle: %.2f", robot.transport.getPivotAngle(), robot.transport.getPivotTargetAngle());
        multipleTelemetry.update();
    }

    public void initialize(){
        robot = new MM_Robot(this);
        robot.init();
        double[] xPoints = robot.drivetrain.navigation.continuousCubicSpline.getxPoints();
        double[] yPoints = robot.drivetrain.navigation.continuousCubicSpline.getyPoints();

        multipleTelemetry.addData("first x point", xPoints[0]);
        multipleTelemetry.addData("second x point", xPoints[1]);
        multipleTelemetry.addData("third x point", xPoints[2]);
        multipleTelemetry.addData("first y point", yPoints[0]);
        multipleTelemetry.addData("second y point", yPoints[1]);
        multipleTelemetry.addData("third y point", yPoints[2]);
        multipleTelemetry.update();
    }
}
