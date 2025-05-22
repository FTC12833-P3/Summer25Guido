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

    private final DcMotorEx flMotor;
    private final DcMotorEx frMotor;
    private final DcMotorEx blMotor;
    private final DcMotorEx brMotor;
    private final Rev2mDistanceSensor backDistance;

    private static final double SLOW_MODE_POWER = .5;
    public static double ROTATE_P_CO_EFF;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    private boolean slowMode = false;

    private double xError = 0;
    private double yError = 0;
    private double headingError = 0;

    public MM_Drivetrain(MM_OpMode opMode){
        this.opMode = opMode;
        navigation = new MM_Navigation(opMode);

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
            flPower = (flPower / maxPower);
            frPower = (frPower / maxPower);
            blPower = (blPower / maxPower);
            brPower = (brPower / maxPower);
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
        xError = navigation.targetPos.getX() - navigation.getX();
        yError = navigation.targetPos.getY() - navigation.getY();
        headingError = getNormalizedHeadingError();

        double moveAngle = Math.toDegrees(Math.atan2(yError, xError));
        double theta = moveAngle - navigation.getHeading() + 45;

        double rotateVector = headingError * ROTATE_P_CO_EFF;
        double strafeVector = Math.cos(Math.toRadians(theta)) - Math.sin(Math.toRadians(theta));//xError * DRIVE_P_COEFF;
        double driveVector = Math.sin(Math.toRadians(theta)) + Math.cos(Math.toRadians(theta));//yError * DRIVE_P_COEFF;

        flPower = driveVector + strafeVector - rotateVector;
        frPower = driveVector - strafeVector + rotateVector;
        blPower = driveVector - strafeVector - rotateVector;
        brPower = driveVector + strafeVector + rotateVector;

        normalize();
        setDrivePowers();

        opMode.multipleTelemetry.addData("zMove angle", moveAngle);
        opMode.multipleTelemetry.addData("zHeading error", headingError);
        opMode.multipleTelemetry.addData("zXError", xError);
        opMode.multipleTelemetry.addData("zYError", yError);
        opMode.multipleTelemetry.addData("zTheta", theta);
    }

    private double getNormalizedHeadingError() {
        double error =  navigation.targetPos.getHeading() - navigation.getHeading();

        error = (error >= 180) ? error - 360 : ((error <= -180) ? error + 360 : error); // a nested ternary to determine error
        return error;
    }

    public double getXError() {
        return xError;
    }

    public double getYError() {
        return yError;
    }

}
