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

    private boolean slowMode = false;
    private boolean reverseMode = false;
    private double PERCENT_TO_SLOW = 0.5;

    // Shooter Motors
    private DcMotor OuttakeFront;
    private Servo pusher;
    private double PUSHER_IN = 0.4;
    private double PUSHER_OUT = 0;

    private double OuttakeFrontPower = 0.6;
    private double OuttakeBackPower = 1;
    private int k = 0;
    private int j = 0;

    // Wobbly boi
    private DcMotor WobbleGrabber;
    float PERCENT_TO_DIVIDE = -0.5f;

    // Intake
    private DcMotor Intake;


    @Override
    public void init() {

        // Drive Initialization
        WheelFrontLeft = hardwareMap.dcMotor.get("Front Left");
        WheelFrontRight = hardwareMap.dcMotor.get("Front Right");
        WheelBackLeft = hardwareMap.dcMotor.get("Back Left");
        WheelBackRight = hardwareMap.dcMotor.get("Back Right");

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);

        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize Shooter
        OuttakeFront = hardwareMap.dcMotor.get("Front Outtake");
        OuttakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeFront.setPower(0);
        OuttakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // TODO: Test shooter motors on FLOAT

        // Initialize pusher
        pusher = hardwareMap.get(Servo.class, "Pusher");
        pusher.setDirection(Servo.Direction.FORWARD);
        pusher.setPosition(PUSHER_IN);

        // Initialize Wobble Grabber
        WobbleGrabber = hardwareMap.dcMotor.get("Wobble Grabber");
        WobbleGrabber.setDirection(DcMotorSimple.Direction.FORWARD);
        WobbleGrabber.setPower(0);
        WobbleGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        // TODO: Need to hook up the 2nd expansion hub before we can update the phones config
//        // Initialize Intake
//        Intake = hardwareMap.dcMotor.get("Intake");
//        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
//        Intake.setPower(0);
//        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    boolean firstClickSlow = true;
    boolean firstClickReverse = true;
    boolean reverseIsOn = false;

    @Override
    public void loop() {

        // **Gamepad 1** \\
        // Drive controls
        float one_right_stick_y = gamepad1.right_stick_y;
        float one_right_stick_x = gamepad1.right_stick_x;
        float one_left_stick_x  = gamepad1.left_stick_x;

        // Buttons
        boolean one_button_a = gamepad1.a;
        boolean one_button_b = gamepad1.b;

        // Bumpers
        boolean one_bumper_left = gamepad1.left_bumper;
        boolean one_bumper_right = gamepad1.right_bumper;

        // **Gamepad 2** \\
        // Wobble Grabber Controls
        float two_right_stick_y = gamepad2.right_stick_y;

        // Buttons
        boolean two_button_a = gamepad2.a;
        boolean two_button_b = gamepad2.b;

        // Bumpers
        boolean two_bumper_right = gamepad2.right_bumper;

        try {

//            // Reverse Mode controls
//            if (one_bumper_right && firstClickReverse){
//                reverseMode = !reverseMode;
//                firstClickReverse = false;
//            }
//
//            if (!one_bumper_right){
//                firstClickReverse = true;
//            }
//
//            // Tell if Reverse Mose is engaged
//            if (reverseMode) {
//                telemetry.addData("REVERSE MODE", "ENGAGED");
//                if (!reverseIsOn) {
//                    one_right_stick_y *= -1;
//                    one_right_stick_x *= -1;
//                    reverseIsOn = true;
//                }
//            } else {
//                telemetry.addData("REVERSE MODE", "DISENGAGED");
//                reverseIsOn = false;
//            }


            // Slow Mode controls
            if (one_bumper_left && firstClickSlow){
                slowMode = !slowMode;
                firstClickSlow = false;
            }

            if (!one_bumper_left){
                firstClickSlow = true;
            }

            // Tell if Slow Mode is engaged
            if (slowMode) {
                telemetry.addData("SLOW MODE", "ENGAGED");
            } else {
                telemetry.addData("SLOW MODE", "DISENGAGED");
            }

            // Drive Code
            ProMotorControl(one_right_stick_y, -one_right_stick_x, -one_left_stick_x);
            telemetry.addData("Left_ x: ", one_left_stick_x);
            telemetry.addData("Right_x: ", one_right_stick_x);
            telemetry.addData("Right_y: ", -one_right_stick_y);

//            // Outtake Power Adjustment
//            if (button_b) {
//                if (k < 1 && j == 0) {          // 20% power
//                    OuttakeFrontPower = 0.2;
//                    j++;
//                } else if (k < 1 && j == 1) {   // 30% power
//                    OuttakeFrontPower = 0.3;
//                    j++;
//                } else if (k < 1 && j == 2) {   // 40% power
//                    OuttakeFrontPower = 0.4;
//                    j++;
//                } else if (k < 1 && j == 3) {   // 50% power
//                    OuttakeFrontPower = 0.5;
//                    j++;
//                } else if (k < 1 && j == 4) {   // 60% power
//                    OuttakeFrontPower = 0.6;
//                    j++;
//                } else if (k < 1 && j == 5) {   // 70% power
//                    OuttakeFrontPower = 0.7;
//                    j++;
//                } else if (k < 1 && j == 6) {   // 80% power
//                    OuttakeFrontPower = 0.8;
//                    j++;
//                } else if (k < 1 && j == 7) {   // 90% power
//                    OuttakeFrontPower = 0.9;
//                    j++;
//                } else if (k < 1 && j == 8) {   // 100% power
//                    OuttakeFrontPower = 1;
//                    j = 0;
//                }
//                k++;
//            } else {
//                k = 0;
//            }

            // Wobbly Boi Code
            MoveWobblyBoi(two_right_stick_y);

            // Outtake Code
            if (two_button_a) {
                OuttakeOn();
            } else {
                OuttakeStop();
            }

            if (two_bumper_right) {
                PushRing();
            } else {
                ResetPusher();
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

        double v1 = r * Math.cos(robotAngle) + leftX;
        double v2 = r * Math.sin(robotAngle) - leftX;
        double v3 = r * Math.sin(robotAngle) + leftX;
        double v4 = r * Math.cos(robotAngle) - leftX;

        if (slowMode) {
            v1 *= PERCENT_TO_SLOW;
            v2 *= PERCENT_TO_SLOW;
            v3 *= PERCENT_TO_SLOW;
            v4 *= PERCENT_TO_SLOW;
        }

        WheelFrontLeft.setPower(v1);
        WheelFrontRight.setPower(v2);
        WheelBackLeft.setPower(v3);
        WheelBackRight.setPower(v4);
    }

    // TODO: Code these functions
    private void OuttakeOn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        // TODO: push the ring into the shooter
        telemetry.addData("Outtake", "ON");
    }

    private void OuttakeStop() {
        OuttakeFront.setPower(0);
        // TODO: move the pusher out of the shooter
        telemetry.addData("Outtake", "OFF");
    }

    private void PushRing() {
        pusher.setPosition(PUSHER_OUT);
    }

    private void ResetPusher() {
        pusher.setPosition(PUSHER_IN);
    }

    private void IntakeStop() {}

    private void IntakeStart() {}

    private void MoveWobblyBoi(float right_stick_y) {
        float armPower = right_stick_y * PERCENT_TO_DIVIDE;

        if (right_stick_y < -0.2 || right_stick_y > 0.2) {
            WobbleGrabber.setPower(armPower);
            telemetry.addData("Status: ", "ON");
        } else {
            WobbleGrabber.setPower(0);
            telemetry.addData("Status: ", "OFF");
        }
        telemetry.addData("Wobbly Boi: ", armPower);
        telemetry.addData("Right Stick Y: ", right_stick_y);
    }

    private void LiftWobble() {}

    private void GrabWobble() {}

    private void DropWobble() {}
}