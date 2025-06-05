package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ascent {
    MM_OpMode opMode;
    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotorEx liftMotor;

    private Servo parkServo;
    public Servo liftRight;
    public Servo liftLeft;

    public boolean lifting = false;
    public boolean pivotInitialPosSet = false;
    public boolean armsMovingUp = false;
    public boolean pivotReadyForLift = false;
    public boolean autoSettingTicks = false;
    public boolean gettingPivotReady = false;
    public int targetPos = 0;


    public MM_Ascent (MM_OpMode opMode){
        this.opMode = opMode;
        parkServo = opMode.hardwareMap.get(Servo.class, "ascentServo");
//        liftRight = opMode.hardwareMap.get(Servo.class, "liftRight");
//        liftLeft = opMode.hardwareMap.get(Servo.class, "liftLeft");
//
//        liftRight.setPosition(.87);
//        liftLeft.setPosition(.13);
//
//
//        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");

        if(opMode.getClass() == MM_Autos.class) {
            parkServo.setPosition(.95);
        }

    }

    public void park(){
        parkServo.setPosition(.1);
    }
}
