package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpTask extends LinearOpMode {

    @Override
    public void runOpMode (){
        DcMotor leftMotor = hardwareMap.get(DcMotor.class,"leftMotor") ;
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"rightMotor") ;

        DcMotorEx leftLinearMotor = hardwareMap.get(DcMotorEx.class, "leftLinearMotor");
        DcMotorEx rightLinearMotor = hardwareMap.get(DcMotorEx.class, "rightLinearMotor");

        Servo leftHookServo = hardwareMap.get(Servo.class, "leftHookServo");
        Servo rightHookServo = hardwareMap.get(Servo.class, "rightHookServo");

        leftMotor.setDirection(DcMotor.Direction.REVERSE) ;

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLinearMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightLinearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double leftPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
            double rightPower = -gamepad1.left_stick_y + gamepad1.right_stick_x;

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            if (gamepad1.x) {
                leftLinearMotor.setPower(1);
                rightLinearMotor.setPower(1);
            };
            if (gamepad1.y) {
                leftLinearMotor.setPower(-1);
                rightLinearMotor.setPower(-1);
            }

            if (gamepad1.left_bumper) {
                leftHookServo.setPosition(0);
                rightHookServo.setPosition(1);
            }
            if (gamepad1.right_bumper) {
                leftHookServo.setPosition(1);
                rightHookServo.setPosition(0);
            }

            telemetry.addData("Left Power:",leftPower) ;
            telemetry.addData("Right Power:",rightPower);
            telemetry.addLine();
            telemetry.addData("Left Linear:",leftLinearMotor.getPower()) ;
            telemetry.addData("Right Linear:",rightLinearMotor.getPower());
            telemetry.addLine();
            telemetry.addData("Left Hook Position:",leftHookServo.getPosition()) ;
            telemetry.addData("Right Hook Position:",rightHookServo.getPosition());
            telemetry.update() ;

        }
    }
}