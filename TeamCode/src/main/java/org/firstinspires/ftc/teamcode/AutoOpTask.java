package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.MotorGroup;

@TeleOp
public class AutoOpTask extends LinearOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class,"leftMotor") ;
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"rightMotor") ;
        DistanceSensor dsensor = hardwareMap.get(DistanceSensor.class,"Distance Sensor");
        MotorGroup linearMotor = new MotorGroup(
                hardwareMap.get(DcMotorEx.class, "leftLinearMotor"),
                hardwareMap.get(DcMotorEx.class, "rightLinearMotor")
        );
        Servo leftHookServo = hardwareMap.get(Servo.class, "leftHookServo");
        Servo rightHookServo = hardwareMap.get(Servo.class, "rightHookServo");

        leftMotor.setDirection(DcMotor.Direction.REVERSE) ;

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            //go straight
            while (true) {
                double distance = dsensor.getDistance(DistanceUnit.CM);

                if (distance <= 3) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                }
/*
                double x = Math.abs(leftMotor.getCurrentPosition());
                double y = Math.abs(rightMotor.getCurrentPosition());
                if (x == 0 && y == 0) {
                    x = y = 1;
                }
                double minimum = Math.min(x, y);
                double leftPower = minimum / Math.abs(leftMotor.getCurrentPosition());
                double rightPower = minimum / Math.abs(rightMotor.getCurrentPosition());
*/
                leftMotor.setPower(1);
                rightMotor.setPower(1);

                telemetry.addData("Left Power:", leftMotor.getPower());
                telemetry.addData("Right Power:", rightMotor.getPower());
                telemetry.addData("Distance:", distance);
                telemetry.update();
            }
            leftHookServo.setPosition(-1);
            rightHookServo.setPosition(1);

            linearMotor.setPower();
        }
    }
}
