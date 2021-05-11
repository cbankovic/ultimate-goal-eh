package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "New TeleOp 2020", group = "Competition")
public class TeleOp01 extends OpMode {

    /**
     * Declare Hardware
     **/

    // Drive Motors
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    private boolean slowMode = false;
    private boolean reverseMode = false;
    private boolean rampUp = true;
    private double PERCENT_TO_SLOW = 0.5;

    // Shooter Motors
    private DcMotor OuttakeFront;
    private Servo pusher;
    private double PUSHER_IN = 0.4;
    private double PUSHER_OUT = 0.2;

    private double OuttakeFrontPower = 0.53;
    private double OuttakeBackPower = 1;
    private int k = 0;
    private int j = 0;

    // Wobbly boi
    private DcMotor WobbleGrabber;
    float PERCENT_TO_DIVIDE = -0.5f;

    // Intake
    private DcMotor IntakeMotor;

    // Voltage
    VoltageSensor voltageSensor;
    double voltage = 0.0;

    private Servo Intake;
    private double PRO_INCREMENT = 0.003;  // amount to slew servo each CYCLE_MS cycle
    private double INCREMENT = 0.1;         // amount to slew servo each CYCLE_MS cycle
    private int CYCLE_MS = 50;              // period of each cycle
    private double MAX_POS = 1.0;           // Maximum rotational position
    private double MIN_POS = 0.0;           // Minimum rotational position
    private double position = 0.0;          // Position to set servo


    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    Shooter shooter;
    double powerShotMultiplier = 0.9875;

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
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize pusher
        pusher = hardwareMap.get(Servo.class, "Pusher");
        pusher.setDirection(Servo.Direction.FORWARD);
        pusher.setPosition(PUSHER_IN);

        // Initialize Wobble Grabber
        WobbleGrabber = hardwareMap.dcMotor.get("Wobble Grabber");
        WobbleGrabber.setDirection(DcMotorSimple.Direction.REVERSE);
        WobbleGrabber.setPower(0);
        WobbleGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Intake
        IntakeMotor = hardwareMap.dcMotor.get("Intake Motor");
        IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setPower(0);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Intake = hardwareMap.get(Servo.class, "Intake");
        Intake.setPosition(position);

        shooter = new Shooter(hardwareMap, telemetry);

        //voltageSensor = (VoltageSensor) hardwareMap.voltageSensor;

    }

    boolean firstClickSlow = true;
    boolean firstClickReverse = true;
    boolean firstClickRamp = true;

    @Override
    public void loop() {

        // **Gamepad 1** \\
        // Drive controls
        float one_right_stick_y = gamepad1.right_stick_y;
        float one_right_stick_x = gamepad1.right_stick_x;
        float one_left_stick_x = gamepad1.left_stick_x;

        // Buttons
        boolean one_button_a = gamepad1.a;
        boolean one_button_b = gamepad1.b;

        // Bumpers
        boolean one_bumper_left = gamepad1.left_bumper;
        boolean one_bumper_right = gamepad1.right_bumper;

        // Triggers
        float one_trigger_left = gamepad1.left_trigger;
        float one_trigger_right = gamepad1.right_trigger;


        // **Gamepad 2** \\
        // Wobble Grabber Controls
        float two_right_stick_y = gamepad2.right_stick_y;

        // Buttons
        boolean two_button_a = gamepad2.a;
        boolean two_button_b = gamepad2.b;
        boolean two_button_x = gamepad2.x;

        // Bumpers
        boolean two_bumper_right = gamepad2.right_bumper;

        // Triggers
        float two_trigger_left = gamepad2.left_trigger;
        float two_trigger_right = gamepad2.right_trigger;


        try {

            // DONE: Make the intake go in reverse to spit out the ring
            // DONE: Make the intake the front of the robot for driving
            // DONE: make the robot intake go up and down via slowly

            // Reverse Mode controls
            if (one_bumper_right && firstClickReverse) {
                reverseMode = !reverseMode;
                firstClickReverse = false;
            }

            if (!one_bumper_right) {
                firstClickReverse = true;
            }

            //telemetry.addData("Intake Position", position);

            // Tell if Reverse Mode is engaged
            if (reverseMode) {
                //telemetry.addData("REVERSE MODE", "ENGAGED");
            } else {
                //telemetry.addData("REVERSE MODE", "DISENGAGED");
            }


            // Intake controls
            if (one_bumper_left && firstClickRamp) {
                rampUp = !rampUp;
                // Retract/Extend Intake
                if (rampUp) {
                    IntakeExtend();
                } else {
                    IntakeRetract();
                }
                firstClickRamp = false;
            }

            if (!one_bumper_left) {
                firstClickRamp = true;
            }

            // Tell if the intake is extended or retracted
            if (rampUp) {
                telemetry.addData("Ramp", "EXTENDED");
            } else {
                telemetry.addData("Ramp", "RETRACTED");
            }


            // Slow Mode controls
            if (one_button_a && firstClickSlow) {
                slowMode = !slowMode;
                firstClickSlow = false;
            }

            if (!one_button_a) {
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
            //telemetry.addData("Left_ x: ", one_left_stick_x);
            //telemetry.addData("Right_x: ", one_right_stick_x);
            //telemetry.addData("Right_y: ", -one_right_stick_y);

            // Intake Code
            IntakeControl(two_trigger_left, two_trigger_right);
            ProServoControl(one_trigger_left, one_trigger_right);

            // Wobbly Boi Code
            MoveWobblyBoi(two_right_stick_y);

            // Outtake Code
            if (two_button_a) {
                OuttakeOn();
            } else if (two_button_x) {
                OuttakePowershot();
            } else if (two_button_b) {
                OuttakeReverse();
            } else {
                OuttakeStop();
            }
//            telemetry.addData("Shooter Timer", shooterTimer.milliseconds());

            if (two_bumper_right) {
                PushRing();
            } else {
                ResetPusher();
            }
        } catch (Exception ex) {
            // Catching the exception
        } finally {
            telemetry.update();
        }
    }


    /**
     * Methods
     **/

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(float right_stick_y, float right_stick_x, float left_stick_x) {
        float powerRightX = right_stick_x;
        float powerRightY = right_stick_y;
        float powerLeftX = left_stick_x;

        if (reverseMode) {
            powerRightX *= -1;
            powerRightY *= -1;
        }

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
        AdjustOuttakeSpeed(1.0);
        telemetry.addData("Outtake", "ON");
    }

    private void OuttakePowershot() {
        AdjustOuttakeSpeed(powerShotMultiplier);
        telemetry.addData("Outtake", "POWERSHOT");
    }

    private void OuttakeReverse() {
        OuttakeFront.setPower(-OuttakeFrontPower);
        telemetry.addData("Outtake", "REVERSE");
    }

    private void OuttakeStop() {
        OuttakeFront.setPower(0);
        telemetry.addData("Outtake", "OFF");
    }

    private void PushRing() {
        pusher.setPosition(PUSHER_OUT);
    }

    private void ResetPusher() {
        pusher.setPosition(PUSHER_IN);
    }

    private void MoveWobblyBoi(float right_stick_y) {
        float armPower = right_stick_y * PERCENT_TO_DIVIDE;

        if (right_stick_y < -0.2 || right_stick_y > 0.2) {
            WobbleGrabber.setPower(armPower);
            telemetry.addData("Status: ", "ON");
        } else {
            WobbleGrabber.setPower(0);
            telemetry.addData("Status: ", "OFF");
        }
        //telemetry.addData("Wobbly Boi: ", armPower);
        //telemetry.addData("Right Stick Y: ", right_stick_y);
    }

    private void IntakeRetract() {

        telemetry.clear();
        //telemetry.addData("Servo", "Getting to starting position...");
        telemetry.update();

        while (Intake.getPosition() != MAX_POS) {

            //telemetry.addData("L Servo", Intake.getPosition());

            position += INCREMENT;
            if (position >= MAX_POS) {
                position = MAX_POS;
            }
            Intake.setPosition(position);
            waitForTime(CYCLE_MS, "Waiting for the servo");
            telemetry.update();
        }
    }

    private void IntakeExtend() {

        telemetry.clear();
        //telemetry.addData("Servo", "Getting to starting position...");
        telemetry.update();

        while (Intake.getPosition() != MIN_POS) {

            //telemetry.addData("L Servo", Intake.getPosition());

            // Keep stepping down until we hit the min value.
            position -= INCREMENT;
            if (position <= MIN_POS) {
                position = MIN_POS;
            }
            Intake.setPosition(position);
            waitForTime(CYCLE_MS, "Waiting for the servo");
            telemetry.update();
        }
    }

    private void ProServoControl(float left_trigger, float right_trigger) {
        if (left_trigger > 0.01 || right_trigger > 0.01) {
            if (left_trigger > 0.01) {
                // Keep stepping up until we hit the max value.
                position += PRO_INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                }
            } else if (right_trigger > 0.01) {
                // Keep stepping down until we hit the min value.
                position -= PRO_INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                }
            }
            Intake.setPosition(position);
        }
    }

    private void IntakeControl(float left_trigger, float right_trigger) {
        if (left_trigger > 0.01) {
            // Keep stepping up until we hit the max value.
            IntakeMotor.setPower(left_trigger);
            //telemetry.addData("Intake Power", left_trigger);
        } else if (right_trigger > 0.01) {
            // Keep stepping up until we hit the max value.
            IntakeMotor.setPower(-right_trigger);
            //telemetry.addData("Intake Power", -right_trigger);
        } else {
            IntakeMotor.setPower(0);
            //telemetry.addData("Intake Power", 0);
        }
    }

    private void waitForTime(int mills, String caption) {
        timer.reset();
        // Wait until the shooter charges up
        while (timer.milliseconds() < mills) {
            // Do nothing
        }
    }

    double OuttakePower;
    private ElapsedTime shooterTimer = new ElapsedTime();
    private double POWER_ADJUST_TIME = 1500;
    private boolean firstTimeAdjust = true;

    private void AdjustOuttakeSpeed(double powerShotMultiplier) {
        if (firstTimeAdjust || shooterTimer.milliseconds() >= POWER_ADJUST_TIME) {
            shooter.AdjustOuttakeSpeed(powerShotMultiplier, false);
            OuttakePower = shooter.OuttakePower;
            SetOuttakePower(OuttakePower);
//        SetOuttakePower(0.50075);
            firstTimeAdjust = false;
            shooterTimer.reset();
        }
        telemetry.addData("OuttakePower", OuttakePower);
    }

    private void SetOuttakePower(double power) {
        OuttakeFront.setPower(power);
    }

}