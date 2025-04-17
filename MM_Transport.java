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
    public final int MAX_SLIDE_TICKS = 3000;
    private final int SLIDE_TICK_INCREMENT = 230;
    private final double PULLEY_DIAMETER = 1.503937;
    private final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
    private final double SLIDE_TICKS_PER_REV = 537.7;
    private final double TICKS_PER_INCH = (SLIDE_TICKS_PER_REV / PULLEY_CIRCUMFERENCE);

    //pivot constants
    private final int PIVOT_TICK_INCREMENT = 150;
    private final double PIVOT_TICKS_PER_REV = 1992.6;
    private final double TICKS_PER_SHAFT_DEGREE = PIVOT_TICKS_PER_REV / 360;
    private final double GEAR_RATIO = 6.0;
    private final double TICKS_PER_PIVOT_DEGREE = TICKS_PER_SHAFT_DEGREE * GEAR_RATIO;
    private final double OFFSET_PIVOT_ANGLE = -25; //Compensation for negative start angle: 25.7223
    private final double MAX_PIVOT_ANGLE = 93;
    private final int MAX_PIVOT_TICKS = pivotDegreesToTicks(MAX_PIVOT_ANGLE); //3819

    private boolean pivotBottomLimitIsHandled = false;

    public MM_Transport(MM_OpMode opMode) {
        this.opMode = opMode;

        pivot = opMode.hardwareMap.get(DcMotorEx.class, "pivot");
        pivotBottomLimit = opMode.hardwareMap.get(TouchSensor.class, "pivotBottomLimit");

        pivot.setDirection(DcMotorEx.Direction.REVERSE);
        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setTargetPosition(0);
        if (opMode.getClass() == MM_Autos.class) {
            pivot.setPower(.85);
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
    }

    public void runPivot(){
        int pivotTargetTicks = 0;

        if(pivotBottomLimit.isPressed() && opMode.gamepad2.left_stick_y >= 0){ //if bottom limit pressed and im not trying to go up
            if(!pivotBottomLimitIsHandled) {
                pivot.setPower(0);
                pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                pivotBottomLimitIsHandled = true;
            }
        } else if(opMode.gamepad2.left_stick_y > .01 || opMode.gamepad2.left_stick_y < .01){
            if(pivotBottomLimitIsHandled){
                pivotBottomLimitIsHandled = false;
                pivot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                pivot.setPower(1);
            }
            pivotTargetTicks = (int)clip(pivot.getCurrentPosition() + -opMode.gamepad2.left_stick_y * PIVOT_TICK_INCREMENT, 0, MAX_PIVOT_TICKS);
        }

        pivot.setTargetPosition(pivotTargetTicks);
    }

    public int getSlideTicks() {
        return slide.getCurrentPosition();
    }

    public double getSlideInches() {
        return slide.getCurrentPosition() / TICKS_PER_INCH;
    }

    public int pivotDegreesToTicks(double degrees) {
        return (int) (TICKS_PER_PIVOT_DEGREE * (degrees - OFFSET_PIVOT_ANGLE));
    }

    public double getPivotDegrees() {
        return (pivot.getCurrentPosition() / TICKS_PER_PIVOT_DEGREE) + OFFSET_PIVOT_ANGLE;
    }

}
