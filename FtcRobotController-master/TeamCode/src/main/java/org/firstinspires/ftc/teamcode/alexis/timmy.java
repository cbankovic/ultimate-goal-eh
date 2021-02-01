package org.firstinspires.ftc.teamcode.alexis;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Chickeb Nugget", group = "Testing")
public class timmy extends OpMode {

    /** Declare Hardware **/
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private CRServo servo;

    private double MotorPower = 0.2;
    private int k = 0;
    private int j = 0;


    @Override
    public void init() {

        motorFrontLeft = hardwareMap.dcMotor.get("Front Left");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setPower(0);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight = hardwareMap.dcMotor.get("Front Right");
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setPower(0);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft = hardwareMap.dcMotor.get("Back Left");
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setPower(0);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackRight = hardwareMap.dcMotor.get("Back Right");
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setPower(0);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        servo = hardwareMap.crservo.get("ServoCont");
        servo = hardwareMap.get(CRServo.class, "ServoCont");

    }

    @Override
    public void loop() {

        boolean ButtonA = gamepad1.a;
        boolean ButtonB = gamepad1.b;
        boolean ButtonY = gamepad1.y;

        try {
            ProMotorControl(gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            telemetry.addData("Left_x", gamepad1.left_stick_x);
            telemetry.addData("Right_x", gamepad1.right_stick_x);
            telemetry.addData("Right_y", -gamepad1.right_stick_y);

            if (ButtonA) {
                servo.setPower(1);
            } else {
                servo.setPower(0);
            }

//            if (ButtonB) {
//                motorBackLeft.setPower(MotorPower);
//            } else {
////                motorBackLeft.setPower(0);
//            }

            if (ButtonY) {
                if (k < 1 && j == 0) {          // 20% power
                    MotorPower = 0.2;
                    j++;
                } else if (k < 1 && j == 1) {   // 30% power
                    MotorPower = 0.3;
                    j++;
                } else if (k < 1 && j == 2) {   // 40% power
                    MotorPower = 0.4;
                    j++;
                } else if (k < 1 && j == 3) {   // 50% power
                    MotorPower = 0.5;
                    j++;
                } else if (k < 1 && j == 4) {   // 60% power
                    MotorPower = 0.6;
                    j++;
                } else if (k < 1 && j == 5) {   // 70% power
                    MotorPower = 0.7;
                    j++;
                } else if (k < 1 && j == 6) {   // 80% power
                    MotorPower = 0.8;
                    j++;
                } else if (k < 1 && j == 7) {   // 90% power
                    MotorPower = 0.9;
                    j++;
                } else if (k < 1 && j == 8) {   // 100% power
                    MotorPower = 1;
                    j = 0;
                }
                k++;
            } else {
                k = 0;
            }

            telemetry.addData("Power", MotorPower * 100 + "%");

        } catch (Exception ex){
            // Catching the exception
        } finally {
            telemetry.update();
        }


    }

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(float right_stick_y, float right_stick_x, float left_stick_x) {
        float powerRightX = right_stick_x;
        float powerRightY = right_stick_y;
        float powerLeftX = left_stick_x;

        double r = Math.hypot(powerRightX, powerRightY);
        double robotAngle = Math.atan2(powerRightY, powerRightX) - Math.PI / 4;
        double leftX = powerLeftX;
        final double v1 = r * Math.cos(robotAngle) + leftX;
        final double v2 = r * Math.sin(robotAngle) - leftX;
        final double v3 = r * Math.sin(robotAngle) + leftX;
        final double v4 = r * Math.cos(robotAngle) - leftX;

        motorFrontLeft.setPower(v1);
        motorFrontRight.setPower(v2);
        motorBackLeft.setPower(v3);
        motorBackRight.setPower(v4);
    }

}
