package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorGroup {
    private final DcMotorEx leftMotor, rightMotor;

    public MotorGroup(DcMotorEx leftMotor, DcMotorEx rightMotor, boolean reverse) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        if (!reverse) rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        else leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public MotorGroup(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this(leftMotor, rightMotor, true);
    }
    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        leftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public double getLeftPower() {
        return leftMotor.getPower();
    }
    public double getRightPower() {
        return rightMotor.getPower();
    }
}
