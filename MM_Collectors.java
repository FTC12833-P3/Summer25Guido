package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collectors {
    MM_OpMode opMode;

    private final double COLLECT_POWER = 1;

    DcMotorEx collectorWheels = null;
    Servo specimenClaw = null;
    ColorRangeSensor innerSampleSensor = null;
    ColorRangeSensor gravityDoorSensor = null;

    public static boolean wheelsCollect = false;
    public static boolean wheelsScore = false;

    private boolean collectStarted = false;
    private boolean scoreStarted = false;

    ElapsedTime collectTimer = new ElapsedTime();
    ElapsedTime scoreTimer = new ElapsedTime();

    ElapsedTime adjustmentTimer = new ElapsedTime();
    boolean adjusting = false;


    public MM_Collectors(MM_OpMode opMode) {
        this.opMode = opMode;

        collectorWheels = opMode.hardwareMap.get(DcMotorEx.class, "wheels");
        specimenClaw = opMode.hardwareMap.get(Servo.class, "specClaw");
        innerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");
        gravityDoorSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "outerSampleLimit");

        collectorWheels.setDirection(DcMotorEx.Direction.REVERSE);
        specimenClaw.setPosition(.5);
    }

    public void runCollectorWheels(){
        if(opMode.gamepad2.right_bumper){
            if(opMode.robot.transport.getPivotCurrentTicks() >= (opMode.robot.transport.getMaxPivotTicks() *.75) || opMode.gamepad2.a){
                collectorWheels.setPower(COLLECT_POWER);
            } else if (haveSample()) {
                collectorWheels.setPower(0);
            } else {
                collectorWheels.setPower(COLLECT_POWER);
            }
        } else if(opMode.gamepad2.left_bumper){
            collectorWheels.setPower(-COLLECT_POWER);
        } else if (adjusting && adjustmentTimer.milliseconds() > 125){
            collectorWheels.setPower(0);
            adjusting = false;
        } else if (!adjusting){
            collectorWheels.setPower(0);
        }

        if (MM_OpMode.currentGamepad2.dpad_down && !MM_OpMode.previousGamepad2.dpad_down){
            collectorWheels.setPower(-.3);
            adjustmentTimer.reset();
            adjusting = true;
        }
        if(MM_OpMode.currentGamepad2.dpad_up && !MM_OpMode.previousGamepad2.dpad_up){
            collectorWheels.setPower(.3);
            adjustmentTimer.reset();
            adjusting = true;
        }
    }

    public void autoRunCollectorWheels(){
        if (wheelsCollect){
            if (!collectStarted) {
                collectTimer.reset();
                collectStarted = !collectStarted;
            }

            if (!haveSample() && collectTimer.milliseconds() < 3000){
                collectorWheels.setPower(COLLECT_POWER);
            } else {
                collectorWheels.setPower(0);
                wheelsCollect = false;
                collectStarted = false;
            }
        } else if (wheelsScore) {
            if (!scoreStarted) {
                scoreTimer.reset();
                scoreStarted = !scoreStarted;
            }

            if (haveSample() || scoreTimer.milliseconds() < 1000) {
                collectorWheels.setPower(COLLECT_POWER);
            } else {
                collectorWheels.setPower(0);
                wheelsScore = false;
                scoreStarted = false;
            }
        }
    }

    public void runSpecClaw(){

    }

    private boolean haveSample(){
        return innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || gravityDoorSensor.getDistance(DistanceUnit.MM) < 67.5;
    }
}