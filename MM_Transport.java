package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MM_Transport {
    private final MM_OpMode opMode;

    private DcMotorEx pivot = null;
    private DcMotorEx slide = null;
    private TouchSensor pivotBottomLimit = null;
    private TouchSensor slideBottomLimit = null;

    //slide constants
    private final int MAX_SLIDE_TICKS = 3000;
    private final int SLIDE_TICK_INCREMENT = 230;
    private final double PULLEY_DIAMETER = 1.503937;
    private final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
    private final double SLIDE_TICKS_PER_REV = 537.7;
    private final double SLIDE_TICKS_PER_INCH = (SLIDE_TICKS_PER_REV / PULLEY_CIRCUMFERENCE);
    private final int MAX_EXTENSION_AT_HORIZONTAL = 17 * (int) SLIDE_TICKS_PER_INCH;
    private final double SLIDE_TICK_THRESHOLD = 50;

    //pivot constants
    private final int PIVOT_TICK_INCREMENT = 150;
    private final double PIVOT_TICKS_PER_REV = 1992.6;
    private final double TICKS_PER_SHAFT_DEGREE = PIVOT_TICKS_PER_REV / 360;
    private final double GEAR_RATIO = 6.0;
    private final double TICKS_PER_PIVOT_DEGREE = TICKS_PER_SHAFT_DEGREE * GEAR_RATIO;
    private final double OFFSET_PIVOT_ANGLE = -25; //Compensation for negative start angle: 25.7223
    private final double MAX_PIVOT_ANGLE = 93;
    private final int MAX_PIVOT_TICKS = pivotDegreesToTicks(MAX_PIVOT_ANGLE); //3819
    private final double PIVOT_THRESHOLD = 100;

    private boolean pivotBottomLimitIsHandled = false;
    private boolean slideHoldingMinimum = false;
    private boolean slideHoming = false;
    private boolean pivotHoming = false;
    private int slideTargetTicks;

    public static double targetPivotAngle = -24;
    public static double slideTargetInches = .5;

    public MM_Transport(MM_OpMode opMode) {
        this.opMode = opMode;

        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");
        pivotBottomLimit = opMode.hardwareMap.get(TouchSensor.class, "pivotBottomLimit");

        pivot.setDirection(DcMotorEx.Direction.REVERSE);
        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setTargetPosition(0);
        if (opMode.getClass() == MM_Autos.class) {
            pivot.setPower(.7);
            pivot.setTargetPosition(1789);
        }
        pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide = opMode.hardwareMap.get(DcMotorEx.class, "slide");
        slideBottomLimit = opMode.hardwareMap.get(TouchSensor.class, "bottomLimit");

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }

    public void runPivot(){
        int pivotTargetTicks = pivot.getTargetPosition();

        if(pivotBottomLimit.isPressed() && opMode.gamepad2.left_stick_y >= 0){ //if bottom limit pressed and im not trying to go up
            if(!pivotBottomLimitIsHandled) {
                pivot.setPower(0);
                pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                pivotBottomLimitIsHandled = true;
            }
        } else if(opMode.gamepad2.left_stick_y > .01 || opMode.gamepad2.left_stick_y < -.01){
            if(pivotBottomLimitIsHandled){
                pivotBottomLimitIsHandled = false;
                pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                pivot.setPower(1);
            }
            pivotTargetTicks = (int)clip(pivot.getCurrentPosition() + -opMode.gamepad2.left_stick_y * PIVOT_TICK_INCREMENT, 0, MAX_PIVOT_TICKS);
            pivotHoming = false;
        } else if (opMode.gamepad2.x) {
            pivotHoming = true;
            pivot.setPower(-0.7);
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        pivot.setTargetPosition(pivotTargetTicks);
    }

    public void autoRunPivot(){
        pivot.setTargetPosition(pivotDegreesToTicks(targetPivotAngle));
    }

    public boolean pivotDone(){
        return Math.abs(pivot.getCurrentPosition() - pivotDegreesToTicks(targetPivotAngle)) < PIVOT_THRESHOLD;
    }

    public void runSlide() {
        if((slideBottomLimit.isPressed() || slideHoldingMinimum) && opMode.gamepad2.right_trigger == 0){ //if bottom limit pressed and im not trying to go up
            if(!slideHoldingMinimum) {
                slideTargetTicks = 10;
                slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                slideHoldingMinimum = true;
                slideHoming = false;
            }
        } else if(opMode.gamepad2.right_trigger > .01 || opMode.gamepad2.left_trigger > .01){
            if(slideHoldingMinimum){
                slideHoldingMinimum = false;
            } else if (slideHoming){
                slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                slideHoming = false;
            }

            if (opMode.gamepad2.right_trigger > 0.01) {
                slideTargetTicks = slide.getCurrentPosition() + (int)(opMode.gamepad2.right_trigger * SLIDE_TICK_INCREMENT);
            } else {
                slideTargetTicks = slide.getCurrentPosition() - (int)(opMode.gamepad2.left_trigger * SLIDE_TICK_INCREMENT);
            }
        } else if (opMode.gamepad2.x &&! slideHoming){
            slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slide.setPower(-.7);
            slideHoming = true;
        } else if (opMode.gamepad2.y && !opMode.gamepad2.b){
            slideTargetTicks = MAX_SLIDE_TICKS;
        }

        if (opMode.gamepad2.y && opMode.gamepad2.b ){
            slideTargetTicks = 1828;
        }

        slide.setTargetPosition(clip(slideTargetTicks, 0, getSlideLimit()));
    }

    public void autoRunSlide(){
        slide.setTargetPosition(Math.min(getSlideLimit(), slideInchesToTicks(slideTargetInches)));
    }

    public void setPivotAngle(double angle){
        pivot.setTargetPosition(pivotDegreesToTicks(angle));
    }

    public boolean slideDone(){
        return Math.abs(slide.getCurrentPosition() - slideInchesToTicks(slideTargetInches)) < SLIDE_TICK_THRESHOLD;
    }

    public int getSlideTicks() {
        return slide.getCurrentPosition();
    }

    public double getSlideInches() {
        return slide.getCurrentPosition() / SLIDE_TICKS_PER_INCH;
    }

    private int getSlideLimit() {
        return (int) Math.min(MAX_SLIDE_TICKS, (MAX_EXTENSION_AT_HORIZONTAL / Math.abs(Math.cos(Math.toRadians(getPivotAngle())))));
    }

    private int slideInchesToTicks(double inches){
        return (int) (inches * SLIDE_TICKS_PER_INCH);
    }

    public double slideTicksToInches(int ticks){
        return (ticks / SLIDE_TICKS_PER_INCH);
    }

    public double getPivotCurrentTicks(){
        return pivot.getCurrentPosition();
    }

    public double getPivotTargetAngle(){
        return pivotTicksToDegrees(pivot.getTargetPosition());
    }

    public double getPivotAngle(){
        return pivotTicksToDegrees(pivot.getCurrentPosition());
    }

    public int getMaxPivotTicks(){
        return MAX_PIVOT_TICKS;
    }

    private double pivotTicksToDegrees(int pivotTicks) {
        return (pivotTicks / TICKS_PER_PIVOT_DEGREE) + OFFSET_PIVOT_ANGLE;
    }

    private int pivotDegreesToTicks(double degrees) {
        return (int) (TICKS_PER_PIVOT_DEGREE * (degrees - OFFSET_PIVOT_ANGLE));
    }

}
