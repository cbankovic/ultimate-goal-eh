package org.firstinspires.ftc.teamcode.christian;

import android.util.SparseArray;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.comp.PIDController;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.List;

//@Disabled
@Autonomous(name = "McTest Vision Auto", group = "Testing")
public class TestAuto01 extends LinearOpMode {

    /************************
     *** DECLARE HARDWARE ***
     ************************/

    // Motors
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;
    //wobbly boi motor
    private DcMotor WobbleGrabber;

    // Shooter Motors
    private DcMotor OuttakeFront;
    private Servo OuttakeBack;
    private double OuttakeFrontPower = 0.2;
    private double OuttakeBackPower = 1;

    // Encoders
    private DcMotor odometerLeft;
    private DcMotor odometerRight;
    private DcMotor odometerBack;
    private DcMotor odometerWobble;

    private int leftCurrentPosition = 0;
    private double leftTargetPosition = 0;
    private int rightCurrentPosition = 0;
    private double rightTargetPosition = 0;
    private int wobbleCurrentPosition = 0;
    private double wobbleTargetPosition = 0;

    private final double COUNTS_PER_INCH = 290; //307.699557;

    // Wobbly Boi Positions
    private float COUNTS_PER_DEGREE = 3.11111111111f;
    private float RETRACTED = 0, OUT = (180 * COUNTS_PER_DEGREE), RAISED = (135 * COUNTS_PER_DEGREE);
    private double WOBBLE_POWER = 0.2;

    // Servos
//    private Servo ServoLeft;
//    private Servo ServoRight;
//    private double INCREMENT = 0.1;     // amount to slew servo each CYCLE_MS cycle
//    private int CYCLE_MS = 50;     // period of each cycle
//    private double MAX_POS = 1.0;     // Maximum rotational position
//    private double MIN_POS = 0.0;     // Minimum rotational position
//    private double position = 0.0;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    // PID
    private double P = .0025, I = 0.0, D = 1.0;
    private PIDController pidRotate, pidDrive;

    // Gyroscope
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private Orientation lastAngles = new Orientation();

    private double globalAngle, power = 0.4, correction, rotation, MAX_POWER = power, MIN_POWER = power * -1;
    private double MIN_CORRECTION = 0.0, MAX_CORRECTION = 0.1;

    enum StrafeDirection {
        Right,
        Left
    }

    // Vision Variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AXbaM4X/////AAABmasKHKdK60IXjDtokRJeu6w/6UeS3KpD2vdyQEvji2tDLEy+IToNeCC4oU0iEAOEzsgw8FABI0qMvJ001KiAfvt3YQETOyjFMY++rqydjE49FLsEPSMGzch3gzelSF9gw3gusCb0rhP/GGXaZnYpN4HbYYI9o/7jUgenQTxlblDFwDsjSgf8TiIoJGTMsW77RCv90nhsWlD+i8qYEUwM3pCxlQ0jImn1+uTTQfoLRNJEn1ZCrDaTcjf5+yxsgdHDXyB5Xh9hd031YFVjX8nX+m9n1ZDHAp8Ha3nLH1MYM5TUuh2/CNZMgyw2BPpAaasW4hT9aDiaYKAVlHQ32dVlTIie2Za4gVFGHgHaahZyUMuz";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**************
     *** SCRIPT ***
     **************/

    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();

        GetGyroInfo();

        SetPIDForward();

        telemetry.addData("=^/", "Waiting for Start 2");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            try {
                if (!opModeIsActive()) {
                    break;
                }

                TurnCameraOn();
                waitForTime(5000, "CAMERA ON");
                //tfod = null;
                //vuforia = null;
                ringCount();
//                TurnCameraOff();
                TurnCameraOff();
                waitForTime(5000, "CAMERA OFF");

//                // Raise the Wobble Goal up
//                moveWobbleForward(RAISED, "raised");
//                // Drive to the ring stack
////                ForwardUntilAtTargetPosition(25);
//                // Detect the ring stack and drive to the appropriate target zone
////                rotate(-90, 0.5);
////                ForwardUntilAtTargetPosition(24);
////                rotate(90, 0.5);
////                timer.reset();
////                while (timer.milliseconds() < 5000) {
////                    if (!opModeIsActive()) {
////                        break;
////                    }
//                TurnCameraOn();
//                    DeliverWobble(ringCount());
////                }
//                TurnCameraOff();
//                waitForTime(5000, "WAITING");
//                // Drop the wobble goal
//                moveWobbleForward(OUT, "out");
////                BackwardUntilAtTargetPosition(5);
//                moveWobbleForward(RETRACTED, "retracted");
//                // Drive to the power shots
////                rotate(90, 0.5);
////                ForwardUntilAtTargetPosition(40);
//                // Drive to the launch line
////                rotate(-90, 0.5);
////                ForwardUntilAtTargetPosition(14);
//                // Shoot
////                ShootRing();
//                // Park on the launch line
////                ForwardUntilAtTargetPosition(5.5);

            } catch (Exception ex) {

                LoadExceptionData(ex, "while");

            } finally {

                telemetry.update();
//                waitForOtherThing();
            }
            break;
        }

    }


    /***************
     *** METHODS ***
     ***************/


    private void waitForOtherThing() {
        while (opModeIsActive()) {
            GetGyroInfo();
            if (!opModeIsActive()) {
                break;
            }
        }
    }

    private void SetPIDForward() {

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(MIN_CORRECTION, MAX_CORRECTION);
        pidDrive.setInputRange(-90, 90);
        pidDrive.setContinuous(true);
        pidDrive.enable();
    }

    private void SetPIDBackward() {

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(-MAX_CORRECTION, -MIN_CORRECTION);
        pidDrive.setInputRange(-90, 90);
        pidDrive.setContinuous(true);
        pidDrive.enable();
    }

    private void Initialize() {

        // Initialize Motors
        telemetry.addData("w", "init wheels");
        WheelFrontLeft = hardwareMap.dcMotor.get("Front Left");
        WheelFrontRight = hardwareMap.dcMotor.get("Front Right");
        WheelBackLeft = hardwareMap.dcMotor.get("Back Left");
        WheelBackRight = hardwareMap.dcMotor.get("Back Right");

        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Shooter
        OuttakeFront = hardwareMap.dcMotor.get("Front Outtake");
        OuttakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeFront.setPower(0);
        OuttakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeBack = hardwareMap.get(Servo.class, "Back Outtake");
        OuttakeBack.setPosition(0);


        //initialize wobbly boi
//        WobbleGrabber
        WobbleGrabber = hardwareMap.dcMotor.get("Wobble Grabber");
        WobbleGrabber.setDirection(DcMotorSimple.Direction.FORWARD);
        WobbleGrabber.setPower(0);
        WobbleGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Encoders
        odometerLeft = hardwareMap.dcMotor.get("Front Left");
        odometerRight = hardwareMap.dcMotor.get("Front Right");
        odometerWobble = hardwareMap.dcMotor.get("Wobble Grabber");

        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerWobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftCurrentPosition = odometerLeft.getCurrentPosition();
        rightCurrentPosition = odometerRight.getCurrentPosition();
        wobbleCurrentPosition = odometerWobble.getCurrentPosition();

//        // Initialize Servos
//        ServoLeft = hardwareMap.get(Servo.class, "LeftHook");
//        ServoRight = hardwareMap.get(Servo.class, "RightHook");
//
//        // Close Servo in preparation to grab the foundation
//        ServoLeft.setPosition(1.0);
//        ServoRight.setPosition(0.0);

        // Initialize IMU
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        /* Set PID proportional value to start reducing power at about 50 degrees of rotation.
         * P by itself may stall before turn completed so we add a bit of I (integral) which
         * causes the PID controller to gently increase power if the turn is not completed. */
        pidRotate = new PIDController(.003, .00003, 0);

        /* Set PID proportional value to produce non-zero correction value when robot veers off
         * straight line. P value controls how sensitive the correction is. */
        pidDrive = new PIDController(P, I, D);

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        resetAngle();

        // Initialize Vision Software
        initVuforia();
        initTfod();

//        if (tfod != null) {
//            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
//            tfod.setZoom(2.5, 1.78);
//        }

        telemetry.addData("Status", "Initialized :D");
        telemetry.update();

    }

    // TODO: Code These Functions
    public void OuttakeOn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        telemetry.addData("Outtake", "ON");
    }
    public void PushRing(){
        OuttakeBack.setPosition(1);
    }
    public void PushReset(){
        OuttakeBack.setPosition(0);
    }

    public void OuttakeStop() {
        OuttakeFront.setPower(0);
        telemetry.addData("Outtake", "OFF");
    }

    private void ShootRing() {
        // Start Shooter
        OuttakeOn();
        waitForTime(5000, "Charging Shooter...");
        PushRing();
        waitForTime(1000, "Shooting Ring");
        // Stop Shooter
        PushReset();
        waitForTime(1000, "Resetting Shooter");
        OuttakeStop();
    }

    private void waitForTime(int mills, String caption) {
        timer.reset();
//        telemetry.addData("W", caption);
//        telemetry.update();
        // Wait until the shooter charges up
        while (timer.milliseconds() < mills) {
            if (!opModeIsActive()) {
                break;
            }
        }
    }

    private void IntakeStop() {}

    private void IntakeStart() {}

    private void CollectRing() {
        // Start Intake
        // Stop Intake
    }

    private void moveWobbleForward(float position, String caption) {

        // Get Current position in ticks
        wobbleCurrentPosition = GetWobblePosition();

        if (wobbleCurrentPosition > position) {
            // Loop until we reach the target
            while (wobbleCurrentPosition > position) {

                if (!opModeIsActive()) {
                    break;
                }

                telemetry.addData("W", "Inside While Loop...");

                // Recalculate the current position
                wobbleCurrentPosition = GetWobblePosition();

                odometerWobble.setPower(-WOBBLE_POWER);

                telemetry.addData("W", "Moving Wobbly Boi");
                telemetry.addData("W", "Position: " + caption);
                telemetry.addData("W", "Current Wobble Position: " + wobbleCurrentPosition);
                telemetry.addData("W", "Target Wobble Position: " + position);
                //LoadTelemetryData();
                telemetry.update();
            }
        } else {
            // Loop until we reach the target
            while (wobbleCurrentPosition < position) {

                if (!opModeIsActive()) {
                    break;
                }

                telemetry.addData("W", "Inside While Loop...");

                // Recalculate the current position
                wobbleCurrentPosition = GetWobblePosition();

                odometerWobble.setPower(WOBBLE_POWER);

                telemetry.addData("W", "Moving Wobbly Boi");
                telemetry.addData("W", "Position: " + caption);
                telemetry.addData("W", "Current Wobble Position: " + wobbleCurrentPosition);
                telemetry.addData("W", "Target Wobble Position: " + position);
//                LoadTelemetryData();
                telemetry.update();
            }

        }

        odometerWobble.setPower(0);
        waitForTime(3000, "Finished " + caption);
    }

    private void moveWobbleBackward(int position) {

        // Get Current position in ticks
        wobbleCurrentPosition = GetWobblePosition();

        position = position * -1;

        // Get Target position by adding current position and # of ticks to travel
        wobbleTargetPosition = wobbleCurrentPosition + calcDistance(position);

        // Loop until we reach the target
        while (leftCurrentPosition > leftTargetPosition) {

            if (!opModeIsActive()) {
                break;
            }

            telemetry.addData("W", "Inside While Loop...");

            // Recalculate the current position
            wobbleCurrentPosition = GetWobblePosition();

            odometerWobble.setPower(0.5);

            telemetry.addData("W", "Moving Wobbly Boi");
            //LoadTelemetryData();
            telemetry.update();
        }

        odometerWobble.setPower(0);

    }

    private void LiftWobble() {}

//    private void GrabWobble() {}

    private void DropWobble() {}

    private void DeliverWobble(String ringStack) {
        if (ringStack == "Quad") {
            // Drive to Target Zone C
            telemetry.addData("RINGS: ", ringStack + " " + 4);
//            ForwardUntilAtTargetPosition(74.25);
        } else if (ringStack ==  "Single") {
            // Drive to Target Zone B
            telemetry.addData("RINGS: ", ringStack + " " + 1);
//            ForwardUntilAtTargetPosition(56.25);
//            rotate(90, 0.5);
//            ForwardUntilAtTargetPosition(24);
//            rotate(-90, 0.5);
        } else {
            // Drive to Target Zone A
            telemetry.addData("RINGS: ", ringStack + " " + 0);
//            ForwardUntilAtTargetPosition(27.75);
        }
        telemetry.update();
        waitForTime(10000, "RingStack");
    }

    private void IdentifyRingNumber() {} // TODO: make int

    private void DriveStraightForwards() {
        correction = pidDrive.performPID(getAngle());

        if (power - correction <= 0.2) {
            power += correction;
        }

        telemetry.addData("LT", "Power Sub    : " + (power - correction));
        telemetry.addData("LT", "Power Add    : " + (power + correction));

        WheelFrontLeft.setPower(power - correction);
        WheelFrontRight.setPower(power + correction);
        WheelBackLeft.setPower(power - correction);
        WheelBackRight.setPower(power + correction);
    }

    private void DriveStraightBackwards() {
        correction = pidDrive.performPID(getAngle());

        if (power + correction >= -0.2) {
            power -= correction;
        }

        telemetry.addData("LT", "Power Sub    : " + (power - correction));
        telemetry.addData("LT", "Power Add    : " + (power + correction));

        WheelFrontLeft.setPower(power - correction);
        WheelFrontRight.setPower(power + correction);
        WheelBackLeft.setPower(power - correction);
        WheelBackRight.setPower(power + correction);
    }

    private void ForwardUntilAtTargetPosition(double distanceInch) {

        power = MAX_POWER;

        SetPIDForward();

        DriveStraightForwards();

        // Get Current position in ticks
        leftCurrentPosition = GetLeftPosition();

        // Get Target position by adding current position and # of ticks to travel
        leftTargetPosition = leftCurrentPosition + calcDistance(distanceInch);

        // Loop until we reach the target
        while (leftCurrentPosition < leftTargetPosition) {

            if (!opModeIsActive()) {
                break;
            }

            telemetry.addData("W", "Inside While Loop...");

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightForwards();

            telemetry.addData("BL", "Back Left: " + GetLeftPosition());
            telemetry.addData("BR", "Back Right: " + GetRightPosition());
            //telemetry.addData("CL", "Current Left", +leftCurrentPosition);
            //telemetry.addData("CR", "Current Right", +rightCurrentPosition);
            //telemetry.addData("TL", "Target Left", +leftTargetPosition);
            //telemetry.addData("TR", "Target Right", +rightTargetPosition);
            telemetry.addData("LT", "Distance: " + calcDistance(distanceInch));
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("correction", correction);
            //telemetry.addData("getP", pidDrive.getP());
            //telemetry.addData("getI", pidDrive.getI());
            ////telemetry.addData("getD", pidDrive.getD());
            //telemetry.addData("getError", pidDrive.getError());
            //LoadTelemetryData();
            telemetry.update();
        }
        BasicMotorControl(0.0);

    }

    private int GetLeftPosition() {
        return odometerLeft.getCurrentPosition();// * -1;
    }

    private int GetRightPosition() {
        return odometerRight.getCurrentPosition();
    }

    private int GetWobblePosition() {
        return odometerWobble.getCurrentPosition();// * -1;
    }
    private void BackwardUntilAtTargetPosition(double distanceInch) {
        power = MIN_POWER;

        SetPIDBackward();

        DriveStraightBackwards();

        distanceInch = distanceInch * -1;

        // Get Current position in ticks
        leftCurrentPosition = GetLeftPosition();

        // Get Target position by adding current position and # of ticks to travel
        leftTargetPosition = leftCurrentPosition + calcDistance(distanceInch);

        // Loop until we reach the target
        while (leftCurrentPosition > leftTargetPosition) {

            if (!opModeIsActive()) {
                break;
            }

            telemetry.addData("W", "Inside While Loop...");

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightBackwards();

            telemetry.addData("LT", "Left Target  : " + leftTargetPosition);
            telemetry.addData("LT", "Left Current : " + leftCurrentPosition);
            telemetry.addData("LT", "Distance: " + calcDistance(distanceInch));
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("correction", correction);
            telemetry.addData("getP", pidDrive.getP());
            telemetry.addData("getI", pidDrive.getI());
            telemetry.addData("getD", pidDrive.getD());
            telemetry.addData("getError", pidDrive.getError());
            //LoadTelemetryData();
            telemetry.update();
        }
        BasicMotorControl(0.0);

    }

    private void ForwardUntilTimerReached(int mills) {

        timer.reset();

        // Loop until we reach the target
        while (timer.milliseconds() < mills) {

            if (!opModeIsActive()) {
                break;
            }

            telemetry.addData("W", "Inside While Loop...");

            // Call BasicMotorControl with new speed
            BasicMotorControl(0.5);
        }

        BasicMotorControl(0.0);

    }

    private double calcDistance(double distanceInch) {
        return COUNTS_PER_INCH * distanceInch;
    }

    //******************************************************************
    // Get the inputs from the controller for power [ BASIC ]
    //******************************************************************
    private void BasicMotorControl(double right_stick_y) {

        WheelFrontLeft.setPower(right_stick_y);
        WheelFrontRight.setPower(right_stick_y);
        WheelBackLeft.setPower(right_stick_y);
        WheelBackRight.setPower(right_stick_y);
    }

//    private void CloseFoundationHook() {
//
//        telemetry.clear();
//        telemetry.addData("Servo", "Getting to starting position...");
//        telemetry.update();
//
//        while (ServoLeft.getPosition() != MAX_POS || ServoRight.getPosition() != MIN_POS) {
//
//            if (!opModeIsActive()) {
//                break;
//            }
//
//            telemetry.addData("L Servo", ServoLeft.getPosition());
//            telemetry.addData("R Servo", ServoRight.getPosition());
//
//            position += INCREMENT;
//            ServoLeft.setPosition(position);
//            ServoRight.setPosition(1 - position);
//            sleep(CYCLE_MS);
//            idle();
//            telemetry.update();
//        }
//    }

//    private void OpenFoundationHook() {
//
//        telemetry.clear();
//        telemetry.addData("Servo", "Getting to starting position...");
//        telemetry.update();
//
//        while (ServoLeft.getPosition() != MIN_POS || ServoRight.getPosition() != MAX_POS) {
//
//            if (!opModeIsActive()) {
//                break;
//            }
//
//            telemetry.addData("L Servo", ServoLeft.getPosition());
//            telemetry.addData("R Servo", ServoRight.getPosition());
//
//            // Keep stepping down until we hit the min value.
//            position += INCREMENT;
//            ServoLeft.setPosition(1 - position);
//            ServoRight.setPosition(position);
//            sleep(CYCLE_MS);
//            idle();
//            telemetry.update();
//        }
//    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        /* Start pid controller. PID controller will monitor the turn angle with respect to the
         * target angle and reduce power as we approach the target angle. This is to prevent the
         * robots momentum from overshooting the turn after we turn off the power. The PID controller
         * reports onTarget() = true when the difference between turn angle and target angle is within
         * 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
         * dependant on the motor and gearing configuration, starting power, weight of the robot and the
         * on target tolerance. If the controller overshoots, it will reverse the sign of the output
         * turning the robot back toward the setpoint value. */

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(.5, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        IMUStuff();

        /* getAngle() returns + when rotating counter clockwise (left) and - when rotating
         * clockwise (right). */

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                WheelFrontLeft.setPower(power);
                WheelBackLeft.setPower(power);
                WheelFrontRight.setPower(-power);
                WheelBackRight.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                WheelFrontLeft.setPower(-power);
                WheelBackLeft.setPower(-power);
                WheelFrontRight.setPower(power);
                WheelBackRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                IMUStuff();
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                WheelFrontLeft.setPower(-power);
                WheelBackLeft.setPower(-power);
                WheelFrontRight.setPower(power);
                WheelBackRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        WheelFrontLeft.setPower(0);
        WheelBackLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackRight.setPower(0);

        rotation = getAngle();

        IMUStuff();

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle() {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private void IMUStuff() {

        telemetry.addData("p", power);
        telemetry.addData("C", correction);
        telemetry.addData("a", getAngle());
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("correction", correction);
        telemetry.addData("getP", pidDrive.getP());
        telemetry.addData("getI", pidDrive.getI());
        telemetry.addData("getD", pidDrive.getD());
        telemetry.addData("getError", pidDrive.getError());
        telemetry.update();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * https://stemrobotics.cs.pdx.edu/node/7268
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
         * We have to process the angle because the imu works in euler angles so the Z axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees. We detect this transition and track the total cumulative angle of rotation. */


        /* Read dimensionalized data from the gyro. This gyro can report angular velocities
         * about all three axes. Additionally, it internally integrates the Z axis to
         * be able to report an absolute angular Z orientation. */
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void GetGyroInfo() {
        modernRoboticsI2cGyro.resetZAxisIntegrator();
    }

    private void LoadExceptionData(Exception ex, String location) {
        telemetry.addData("Exception Location: ", location);
        telemetry.addData("Exception ToString", ex.toString());
        telemetry.addData("Message", ex.getMessage());
        telemetry.addData("GetCause", ex.getCause());
        telemetry.addData("Line Number", ex.getStackTrace()[0].getLineNumber());
        telemetry.addData("Class", ex.getClass().getCanonicalName());

        StringWriter sw = new StringWriter();
        ex.printStackTrace(new PrintWriter(sw));
        String exceptionAsString = sw.toString();
        telemetry.addData("StackTrace", exceptionAsString);
    }

    private void StrafeUntilTimerReached(StrafeDirection strafeDirection, double power, int mills) {

        timer.reset();

        // Loop until we reach the target
        while (timer.milliseconds() < mills) {

            if (!opModeIsActive()) {
                break;
            }

            telemetry.addData("W", "Inside While Loop...");

            // Call StrafeMotorControl with new speed

            if (strafeDirection == StrafeDirection.Left) {
                ProMotorControl(-0.1, -power, 0.0);
            } else if (strafeDirection == StrafeDirection.Right) {
                ProMotorControl(-0.1, power, 0.0);
            } else {
                telemetry.addData("          ERROR:", "Valid strafing direction not specified.");
                telemetry.addData("TROUBLESHOOTING:", "Check if direction parameter is capitalized properly.");
                telemetry.update();
            }
        }

        ProMotorControl(0.0, 0.0, 0.0);
    }

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(double right_stick_y, double right_stick_x, double left_stick_x) {
        double powerRightY = right_stick_y;
        double powerRightX = right_stick_x;
        double powerLeftX = left_stick_x;

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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private String ringCount() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {

                    float right = recognition.getBottom();
                    float left = recognition.getTop();
                    float top = recognition.getRight();
                    float bottom = recognition.getLeft();

                    float height = Math.abs( top - bottom);
                    float width = Math.abs(right - left);
                    float area = Math.abs(height * width);
                    float xCenter = (right + left) / 2;
                    float yCenter = (top + bottom) / 2;

                    // 1280 x 720 = 720p Resolution
                    // 640 (x) x 360 (y) = center
                    String horizontal = (xCenter < 620  ? "Left" : (xCenter > 660 ? "Right" : "Good"));
                    String vertical = (yCenter < 350  ? "Down" : (yCenter > 370 ? "Up" : "Good"));

                    String ringLabel = recognition.getLabel();
                    telemetry.addData(String.format("label (%d)", i), ringLabel);
                    telemetry.addData(String.format("  top, left (%d)", i), "%.03f , %.03f",
                            top, left);
                    telemetry.addData(String.format("  bottom, right (%d)", i), "%.03f , %.03f",
                            bottom , right);
                    telemetry.addData(String.format("  height, width (%d)", i), "%.03f , %.03f",
                            height, width);
                    telemetry.addData(String.format("  area (%d)", i), "%.03f ",area);
                    telemetry.addData(String.format("  xC, yC (%d)", i), "%.03f , %.03f",
                            xCenter, yCenter);
                    telemetry.addData(String.format("  horizon, vert (%d)", i), "%s, %s", horizontal, vertical);


                    if (area <= 24000){
                        telemetry.addData("Distance", "14 inches");
                    } else if (area <= 42000){
                        telemetry.addData("Distance", "12 inches");
                    } else if (area <= 59000){
                        telemetry.addData("Distance", "10 inches");
                    } else if (area <= 99000){
                        telemetry.addData("Distance", "8 inches");
                    } else if (area <= 105000){
                        telemetry.addData("Distance", "6 inches");
                    }
                    return ringLabel;
                }
                telemetry.update();
            }
        }
        return null;
    }

    private void TurnCameraOff() {
        vuforia.getCamera().close();
    }

    private  void TurnCameraOn() {
        tfod.activate();
    }

}