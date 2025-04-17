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
        }

        runProcedures();
    }

    public abstract void runProcedures();

    public void telemetry(){
        multipleTelemetry.addData("slide", "inches: %.2f slide ticks: %d", robot.transport.getSlideInches(), robot.transport.getSlideTicks());
        multipleTelemetry.addData("pivot", "degrees: %.2f", robot.transport.getPivotDegrees());
        multipleTelemetry.update();
    }

    public void initialize(){
        robot = new MM_Robot(this);
        robot.init();
    }
}
