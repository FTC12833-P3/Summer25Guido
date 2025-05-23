package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MM_OpMode.currentGamepad1;
import static org.firstinspires.ftc.teamcode.MM_OpMode.previousGamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
public class MM_Drivetrain {
    MM_OpMode opMode;
    MM_Navigation navigation;
    MM_PID_CONTROLLER pidController;

    private final DcMotorEx flMotor;
    private final DcMotorEx frMotor;
    private final DcMotorEx blMotor;
    private final DcMotorEx brMotor;
    private final Rev2mDistanceSensor backDistance;

    private static final double SLOW_MODE_POWER = .5;
    public static double ROTATE_P_CO_EFF;
    private final double X_ERROR_THRESHOLD = 1;
    private final double Y_ERROR_THRESHOLD = 1;
    private final double HEADING_ERROR_THRESHOLD = 3;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    private boolean slowMode = false;
    public static double desiredPower = 1;

    private double xError = 0;
    private double yError = 0;
    private double headingError = 0;

    public MM_Drivetrain(MM_OpMode opMode){
        this.opMode = opMode;
        navigation = new MM_Navigation(opMode);
        pidController = new MM_PID_CONTROLLER(0.5, 0, 50);

        flMotor = opMode.hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = opMode.hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = opMode.hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = opMode.hardwareMap.get(DcMotorEx.class, "brMotor");

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);

        backDistance = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
    }

    public void driveWithSticks() {
        double drivePower = -opMode.gamepad1.left_stick_y;
        double strafePower = opMode.gamepad1.left_stick_x;
        double rotatePower = -opMode.gamepad1.right_stick_x;

        if (currentGamepad1.a && !previousGamepad1.a && !currentGamepad1.start) {
            slowMode = !slowMode;
        }

        flPower = drivePower + strafePower - rotatePower;
        frPower = drivePower - strafePower + rotatePower;
        blPower = drivePower - strafePower - rotatePower;
        brPower = drivePower + strafePower + rotatePower;
        setDrivePowers();
    }

    private void normalize() {
        double maxPower = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

        if (maxPower > 1) {
            flPower = (flPower / maxPower) * desiredPower;
            frPower = (frPower / maxPower) * desiredPower;
            blPower = (blPower / maxPower) * desiredPower;
            brPower = (brPower / maxPower) * desiredPower;
        }
    }

    private void setDrivePowers() {
        normalize();

        if (slowMode) {
            flPower *= SLOW_MODE_POWER;
            frPower *= SLOW_MODE_POWER;
            blPower *= SLOW_MODE_POWER;
            brPower *= SLOW_MODE_POWER;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void autoRunDrivetrain() {
        navigation.updatePosition();
        xError = MM_Navigation.targetPos.getX() - navigation.getX();
        yError = MM_Navigation.targetPos.getY() - navigation.getY();
        headingError = getNormalizedHeadingError();

        double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
        double theta = moveAngle - navigation.getHeading() + 45;

        //double rotateVector = headingError * ROTATE_P_CO_EFF;
        double PID = pidController.getPID(Math.hypot(xError, yError));

        flPower = 2 * Math.cos(Math.toRadians(theta)) * PID;
        frPower = 2 * Math.sin(Math.toRadians(theta)) * PID;
        blPower = frPower; //I double checked these lines.
        brPower = flPower; // It turns out that bl power is always the same as fr power and same for the other two, without rotate

        normalize();
        setDrivePowers();

        opMode.multipleTelemetry.addData("zMove angle", moveAngle);
        opMode.multipleTelemetry.addData("zHeading error", headingError);
        opMode.multipleTelemetry.addData("zXError", xError);
        opMode.multipleTelemetry.addData("zYError", yError);
        opMode.multipleTelemetry.addData("zTheta", theta);
    }

    public boolean driveDone() {
        return xError < X_ERROR_THRESHOLD && yError < Y_ERROR_THRESHOLD && headingError < HEADING_ERROR_THRESHOLD;
    }

    private double getNormalizedHeadingError() {
        double error =  MM_Navigation.targetPos.getHeading() - navigation.getHeading();

        error = (error >= 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

}
