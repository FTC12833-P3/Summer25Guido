package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ascent {
    MM_OpMode opMode;
    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotorEx liftMotor;

    public static double TICK_INCREMENT = 100;


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
        liftRight = opMode.hardwareMap.get(Servo.class, "liftRight");
        liftLeft = opMode.hardwareMap.get(Servo.class, "liftLeft");

        liftRight.setPosition(.87);
        liftLeft.setPosition(.13);

        liftMotor = opMode.hardwareMap.get(DcMotorEx.class, "liftMotor");

        if(opMode.getClass() == MM_Autos.class) {
            parkServo.setPosition(.95);
        }

    }

    public void park(){
        parkServo.setPosition(.08);
    }

    public void controlAscent(){
        if(MM_OpMode.currentGamepad1.y && !MM_OpMode.previousGamepad1.y){
            parkServo.setPosition(.08);
        }

        if(opMode.gamepad1.b){
            parkServo.setPosition(.95);
        }

        if(opMode.getClass() == MM_TeleOp.class && (opMode.gamepad1.dpad_up || lifting)){
            lifting = true;
            if(!pivotInitialPosSet) {
                opMode.robot.transport.setPivotAngle(90);
                pivotInitialPosSet = true;
            }
            if(opMode.robot.transport.getPivotAngle() >=70 && !armsMovingUp) {
                liftTimer.reset();
                liftLeft.setPosition(.57);
                liftRight.setPosition(.43);
                armsMovingUp = true;
            }

            if(liftTimer.milliseconds() > 1000 && armsMovingUp){
                if(Math.abs(liftMotor.getCurrentPosition()) >= 450) {
                    opMode.robot.transport.setPivotAngle(40);
                    if (opMode.robot.transport.getPivotAngle() <= 45) {
                        pivotReadyForLift = true;
                    }
                }
                liftLeft.getController().pwmDisable();
                liftRight.getController().pwmDisable();
            }
            if(armsMovingUp && liftTimer.milliseconds() > 1200) {
                if(!autoSettingTicks && pivotReadyForLift){
                    autoSettingTicks = true;
                    targetPos = -7250;
                } else if(!gettingPivotReady){
                    gettingPivotReady = true;
                    targetPos = -500;
                }

                if(autoSettingTicks) {
                    if(opMode.gamepad1.right_trigger > 0) {
                        targetPos = (int) (liftMotor.getCurrentPosition() + (opMode.gamepad1.right_trigger * TICK_INCREMENT));
                    } else if (opMode.gamepad1.left_trigger > 0) {
                        targetPos = Math.max((int) (liftMotor.getCurrentPosition() - (opMode.gamepad1.left_trigger * TICK_INCREMENT)), -7300);
                    }
                }
                if (liftMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                    liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                }

                liftMotor.setTargetPosition(targetPos);
            }
            opMode.multipleTelemetry.addData("lift ticks", liftMotor.getCurrentPosition());
            opMode.multipleTelemetry.update();
        }
    }

    public void fixString(){
        liftLeft.getController().pwmDisable();
        liftRight.getController().pwmDisable();

        if(opMode.gamepad1.right_trigger > 0) {
            targetPos = (int) (liftMotor.getCurrentPosition() + (opMode.gamepad1.right_trigger * TICK_INCREMENT));
        } else if (opMode.gamepad1.left_trigger > 0) {
            targetPos = Math.max((int) (liftMotor.getCurrentPosition() - (opMode.gamepad1.left_trigger * TICK_INCREMENT)), -7300);
        }
        if (liftMotor.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
            liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }

        liftMotor.setTargetPosition(targetPos);
    }
}
