package org.firstinspires.ftc.teamcode;

public class Constant {
    //All this are pseudo-constant
    static final public double leftKP = 2;
    static final public double rightKP = 2.2;
    static final public double leftKI = 0.0005;
    static final public double rightKI = 0.0013;
    static final public double leftKD = 0.027;
    static final public double rightKD = 0.035;

    static final public double HD_COUNTS_PER_REV = 28;
    static final public double DRIVE_GEAR_REDUCTION = 20.15293;
    static final public double EXTEND_LENGTH_PER_REV_MM = 90;
    static final public double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / EXTEND_LENGTH_PER_REV_MM;
}
