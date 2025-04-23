package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collectors {
    MM_OpMode opMode;

    private final double COLLECT_POWER = 1;

    DcMotorEx collectorWheels = null;
    Servo specimenClaw = null;
    ColorRangeSensor innerSampleSensor = null;
    ColorRangeSensor gravityDoorSensor = null;

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
        } else if(outtakeTimer.milliseconds() < 125) {
            collectorWheels.setPower(-.3);
        } else if (collectTime.milliseconds() < 125) {
            collectorWheels.setPower(.3);
        } else {
            collectorWheels.setPower(0);

        }

        if ( MM_OpMode.currentGamepad2.dpad_down && !MM_OpMode.previousGamepad2.dpad_down){
            outtakeTimer.reset();
        }
        if(MM_OpMode.currentGamepad2.dpad_up && !MM_OpMode.previousGamepad2.dpad_up){
            collectTime.reset();
        }
    }

    private boolean haveSample(){
        return innerSampleSensor.getDistance(DistanceUnit.MM) < 60 || gravityDoorSensor.getDistance(DistanceUnit.MM) < 67.5;
    }
}