package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    // Elapsed timer class from SDK, please use it, it's epic
    private double kP, kI, kD;
    private DcMotorEx motor;

    public PID(DcMotorEx motor, double kP, double kI, double kD) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void control(double target) {
        ElapsedTime timer = new ElapsedTime();
        double integralSum = 0;
        double lastError = 0;

        while (motor.getCurrentPosition() < target) {
            double error = target - motor.getCurrentPosition();

            double derivative = (error - lastError) / timer.seconds();

            if(Math.abs(error) < 200 && error !=0){
                integralSum += error * timer.seconds();
            }
            else{
                integralSum = 0;
            }


            double out = (kP * error) + (kI * integralSum) + (kD * derivative);
            motor.setPower(out);

            lastError = error;
            timer.reset();
        }
    }
}

