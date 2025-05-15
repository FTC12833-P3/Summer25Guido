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

    public static boolean collect = false;

    ElapsedTime adjustmentTimer = new ElapsedTime();
    boolean adjusting = false;


    public MM_Collectors(MM_OpMode opMode) {
        this.opMode = opMode;

        collectorWheels = opMode.hardwareMap.get(DcMotorEx.class, "wheels");
        specimenClaw = opMode.hardwareMap.get(Servo.class, "specClaw");
        innerSampleSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "sampleLimit");
        gravityDoorSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "outerSampleLimit");

        collectorWheels.setDirection(DcMotorEx.Direction.REVERSE);
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

        if ( MM_OpMode.currentGamepad2.dpad_down && !MM_OpMode.previousGamepad2.dpad_down){
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
        if (collect){
            if (!haveSample()){
                collectorWheels.setPower(COLLECT_POWER);
            } else {
                collectorWheels.setPower(0);
                collect = false;
            }
        }
    }

    private boolean haveSample(){
        return innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || gravityDoorSensor.getDistance(DistanceUnit.MM) < 67.5;
    }

}