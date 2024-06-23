package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constant.DRIVE_COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.Constant.leftKD;
import static org.firstinspires.ftc.teamcode.Constant.leftKI;
import static org.firstinspires.ftc.teamcode.Constant.leftKP;
import static org.firstinspires.ftc.teamcode.Constant.rightKD;
import static org.firstinspires.ftc.teamcode.Constant.rightKI;
import static org.firstinspires.ftc.teamcode.Constant.rightKP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.PID;

@TeleOp
public class AutoOpTask extends LinearOpMode {

    private double mmToTicks(double mm) {
        return mm * DRIVE_COUNTS_PER_MM;
    }

    @Override
    public void runOpMode () throws InterruptedException {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class,"leftMotor") ;
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"rightMotor") ;
        DistanceSensor dsensor = hardwareMap.get(DistanceSensor.class,"Distance Sensor");
        DcMotorEx leftElevatorMotor = hardwareMap.get(DcMotorEx.class, "leftElevatorMotor");
        DcMotorEx rightElevatorMotor = hardwareMap.get(DcMotorEx.class, "rightElevatorMotor");
        Servo leftBarServo = hardwareMap.get(Servo.class, "leftBarServo");
        Servo rightBarServo = hardwareMap.get(Servo.class, "rightBarServo");

        leftMotor.setDirection(DcMotor.Direction.REVERSE) ;

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PID leftLinearControl = new PID(leftElevatorMotor, leftKP, leftKI, leftKD);
        PID rightLinearControl = new PID(rightElevatorMotor, rightKP, rightKI, rightKD);

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

                leftMotor.setPower(1);
                rightMotor.setPower(1);

                telemetry.addData("Left Power:", leftMotor.getPower());
                telemetry.addData("Right Power:", rightMotor.getPower());
                telemetry.addData("Distance:", distance);
                telemetry.update();
            }

            leftBarServo.setPosition(0.5);
            rightBarServo.setPosition(0.5);

            leftLinearControl.control(mmToTicks(400));
            rightLinearControl.control(mmToTicks(-400));

            leftMotor.setPower(1);
            rightMotor.setPower(1);
            sleep(2500);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            leftLinearControl.control(mmToTicks(400));
            rightLinearControl.control(mmToTicks(-400));
        }
    }
}
