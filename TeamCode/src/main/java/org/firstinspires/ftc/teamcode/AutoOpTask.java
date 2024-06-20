package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AutoOpTask extends LinearOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class,"leftMotor") ;
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"rightMotor") ;
        DistanceSensor dsensor = hardwareMap.get(DistanceSensor.class,"Distance Sensor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.REVERSE) ;

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            double distance = dsensor.getDistance(DistanceUnit.CM) ;

            if ( distance <= 3 ) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break ;
            }

            double x = Math.abs(leftMotor.getCurrentPosition());
            double y = Math.abs(rightMotor.getCurrentPosition());
            if ( x == 0 && y == 0 ) {
                x = y = 1 ;
            }
            double minimum = Math.min(x,y) ;
            double leftPower = minimum / Math.abs(leftMotor.getCurrentPosition()) ;
            double rightPower = minimum / Math.abs(rightMotor.getCurrentPosition());

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            telemetry.addData("Left Power:",leftPower) ;
            telemetry.addData("Right Power:",rightPower);
            telemetry.addData("Distance:",distance) ;
            telemetry.update() ;

        }
    }
}
