package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "New TeleOp 2020", group = "Competition")
public class TeleOp01 extends OpMode {

    /** Declare Hardware **/

    // Drive Motors
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    // Shooter Motors
    private DcMotor OuttakeFront;
    private Servo OuttakeBack;
    private double OuttakeFrontPower = 0.2;
    private double OuttakeBackPower = 1;
    private int k = 0;
    private int j = 0;


    @Override
    public void init() {

        // Drive Initialization
        WheelFrontLeft = hardwareMap.dcMotor.get("Front Left");
        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontLeft.setPower(0);
        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        WheelFrontRight = hardwareMap.dcMotor.get("Front Right");
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setPower(0);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        WheelBackLeft = hardwareMap.dcMotor.get("Back Left");
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setPower(0);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        WheelBackRight = hardwareMap.dcMotor.get("Back Right");
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setPower(0);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Shooter Initialization
        OuttakeFront = hardwareMap.dcMotor.get("Front Outtake");
        OuttakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeFront.setPower(0);
        OuttakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // TODO: Test shooter motors on FLOAT

//        OuttakeBack = hardwareMap.dcMotor.get("Back Outtake");
//        OuttakeBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        OuttakeBack.setPower(0);
//        OuttakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        OuttakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        OuttakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        // Drive controls
        float right_stick_y = gamepad1.right_stick_y;
        float right_stick_x = gamepad1.right_stick_x;
        float left_stick_x  = gamepad1.left_stick_x;

        // Buttons
        boolean button_a = gamepad1.a;
        boolean button_b = gamepad1.b;

        try {
            // Drive Code
            ProMotorControl(right_stick_y, -right_stick_x, -left_stick_x);
            telemetry.addData("Left_ x: ", left_stick_x);
            telemetry.addData("Right_x: ", right_stick_x);
            telemetry.addData("Right_y: ", -right_stick_y);

            // Outtake Power Adjustment
            if (button_b) {
                if (k < 1 && j == 0) {          // 20% power
                    OuttakeFrontPower = 0.2;
                    j++;
                } else if (k < 1 && j == 1) {   // 30% power
                    OuttakeFrontPower = 0.3;
                    j++;
                } else if (k < 1 && j == 2) {   // 40% power
                    OuttakeFrontPower = 0.4;
                    j++;
                } else if (k < 1 && j == 3) {   // 50% power
                    OuttakeFrontPower = 0.5;
                    j++;
                } else if (k < 1 && j == 4) {   // 60% power
                    OuttakeFrontPower = 0.6;
                    j++;
                } else if (k < 1 && j == 5) {   // 70% power
                    OuttakeFrontPower = 0.7;
                    j++;
                } else if (k < 1 && j == 6) {   // 80% power
                    OuttakeFrontPower = 0.8;
                    j++;
                } else if (k < 1 && j == 7) {   // 90% power
                    OuttakeFrontPower = 0.9;
                    j++;
                } else if (k < 1 && j == 8) {   // 100% power
                    OuttakeFrontPower = 1;
                    j = 0;
                }
                k++;
            } else {
                k = 0;
            }

            // Outtake Code
            if (button_a) {
                OuttakeOn();
            } else {
                OuttakeStop();
            }

        } catch (Exception ex){
            // Catching the exception
        } finally {
            telemetry.update();
        }


    }


    /** Methods **/

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

        WheelFrontLeft.setPower(v1);
        WheelFrontRight.setPower(v2);
        WheelBackLeft.setPower(v3);
        WheelBackRight.setPower(v4);
    }

    // TODO: Code these functions
    public void OuttakeOn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        // TODO: push the ring into the shooter
        telemetry.addData("Outtake", "ON");
    }

    public void OuttakeStop() {
        OuttakeFront.setPower(0);
        // TODO: move the pusher out of the shooter
        telemetry.addData("Outtake", "OFF");
    }

    private void IntakeStop() {}

    private void IntakeStart() {}

    private void LiftWobble() {}

    private void GrabWobble() {}

    private void DropWobble() {}
}