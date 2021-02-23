package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.List;

@Autonomous(name = "Auto01 2020", group = "Competition")
public class Auto01 extends LinearOpMode {

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
    private Servo pusher;
    private double PUSHER_IN = 0.4;
    private double PUSHER_OUT = 0;

    private double OuttakeFrontPower = 0.65;
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
    private int backCurrentPosition = 0;
    private double backTargetPosition = 0;
    private int wobbleCurrentPosition = 0;
    private double wobbleTargetPosition = 0;

    private final double COUNTS_PER_INCH = 290; //307.699557;

    // Wobbly Boi Positions
    private float COUNTS_PER_DEGREE = 3.11111111111f;
    private float RETRACTED = 0, OUT = (80 * COUNTS_PER_DEGREE), RAISED = (135 * COUNTS_PER_DEGREE);
    private double WOBBLE_POWER = 0.50;

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
    //private double P = .0025, I = 0.0, D = 1.0;
    private double P = .0025, I = 0.02, D = 0.01;
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

    private enum RingsFound {
        None,
        Single,
        Quad,
        NotFound
    }

    /**************
     *** SCRIPT ***
     **************/

    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();

        GetGyroInfo();

        SetPIDForward();

        // TODO: uncomment to wait for the camera to activate
        //sleep(3000);
        telemetry.addData("=D", "Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            try {
                if (!opModeIsActive()) {
                    break;
                }

                // TODO: uncomment
                //RingsFound ringLocation = FindRings();
                // TODO: delete line - TESTING ONLY
                RingsFound ringLocation = RingsFound.Single;

                // Detect the ring stack and drive to the appropriate target zone
                switch (ringLocation) {
                    case None:
                        telemetry.addData("Rings found: ", "None");
                        DriveToA();
                        break;
                    case Single:
                        telemetry.addData("Rings found: ", "Single");
                        DriveToB();
                        break;
                    case Quad:
                        telemetry.addData("Rings found: ", "Quad");
                        DriveToC();
                        break;
                    default:
                        telemetry.addData("Rings found: ", "NULL VALUE");
                        // Issue with vision - we randomly select target zone B
                        // May the odds forever be in your favor
                        // We have to navigate around the stack if there is one present
                        ringLocation = RingsFound.Single;
                        DriveToB();
                        break;
                }

                ShootRing();
                ShootRing();
                ShootRing();
                OuttakeStop();

                // Detect the ring stack and drive to the appropriate target zone
                switch (ringLocation) {
                    case None:
                        // Rotate to drop the wobble goal
                        rotate(-30, 0.4);
                        rotate(-30, 0.4);

                        // Drop the wobble goal
                        PlaceWobbleGoal();

                        // Rotate before parking on the line
                        rotate(30, 0.5);
                        ForwardUntilAtTargetPosition(18);
                        break;
                    case Single:
                        // Rotate before parking on the line
                        rotate(20, 0.5);
                        ForwardUntilAtTargetPosition(18);
                        break;
                    case Quad:
                        // Rotate before parking on the line
                        rotate(20, 0.5);
                        ForwardUntilAtTargetPosition(18);
                        break;
                    default:
                        telemetry.addData("Rings found: ", "NULL VALUE");
                        break;
                }
            } catch (Exception ex) {

                LoadExceptionData(ex, "while");

            } finally {
                telemetry.update();
            }
            break;
        }

        TurnOffCamera();
    }

    private void DriveToC() {

        // Drive for distance to the C dropoff point
        ForwardUntilAtTargetPosition(5);

        // Rotate to get around the ring stack
        rotate(-20, 0.4);

        // Drive past the ring stack
        ForwardUntilAtTargetPosition(50);

        // Rotate to drive to the C target zone
        rotate(20, 0.4);

        // Drive to the target zone
        ForwardUntilAtTargetPosition(45);

        // Drop off wobble goal
        PlaceWobbleGoal();

        // Start shooter motor
        StartShooter();

        // Drive to shooting position
        rotate(-20, 0.4);
        BackwardUntilAtTargetPosition(42);
        rotate(45, 0.4);
        rotate(18, 0.4);
    }

    private void DriveToB() {
        // Drive for distance before rotating
        ForwardUntilAtTargetPosition(5);

        // Rotate to get around the ring stack
        rotate(-20, 0.4);

        // Drive past the ring stack
        ForwardUntilAtTargetPosition(50);

        // Rotate towards target zone B
        rotate(30, 0.4);
        rotate(25, 0.4);

        // Drive to target zone B
        ForwardUntilAtTargetPosition(20);

        // Place wobble goal
        PlaceWobbleGoal();

        // Rotate to get around the ring stack
        rotate(-30, 0.4);
        rotate(-30, 0.4);

        // Start shooter motor
        StartShooter();

        // Drive to the shooting location
        BackwardUntilAtTargetPosition(17);

        // Rotate robot to shoot in the high goal
        rotate(30, 0.5);
        rotate(18, 0.5);
    }

    private void DriveToA() {
        // Turn on the shooter motor
        StartShooter();

        // Drive to the shooting location
        ForwardUntilAtTargetPosition(55);

        // Rotate robot to shoot in the high goal
        rotate(18, 0.5);
    }

    private void PlaceWobbleGoal() {
        moveWobbleForward(OUT, "OUT");
        sleep(200);
        moveWobbleForward(RETRACTED, "RETRACTED");
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
        telemetry.update();


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

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // Initialize pusher
        pusher = hardwareMap.get(Servo.class, "Pusher");
        pusher.setDirection(Servo.Direction.FORWARD);
        pusher.setPosition(PUSHER_IN);

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
        odometerBack = hardwareMap.dcMotor.get("Back Right");
        odometerWobble = hardwareMap.dcMotor.get("Wobble Grabber");

        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerWobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometerWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftCurrentPosition = odometerLeft.getCurrentPosition();
        rightCurrentPosition = odometerRight.getCurrentPosition();
        backCurrentPosition = odometerBack.getCurrentPosition();
        wobbleCurrentPosition = odometerWobble.getCurrentPosition();

        // Initialize IMU
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        telemetry.addData("GC", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
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
            if (!opModeIsActive()) {
                break;
            }
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        resetAngle();

        // Initialize Vision Software
        initVuforia();
        initTfod();

        if (tfod != null) {
            TurnOnCamera();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.7777);
            //tfod.setClippingMargins(350, 150, 350, 200);
        }

        telemetry.addData("Status", "Initializing camera");
        telemetry.update();
    }

    private void StartShooter() {
        // Start Shooter
        OuttakeOn();
        //waitForTime(3500, "Charging Shooter...");
    }

    public void PushRing() {
        pusher.setPosition(PUSHER_OUT);
    }

    public void PushReset() {
        pusher.setPosition(PUSHER_IN);
    }

    public void OuttakeOn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        telemetry.addData("Outtake", "ON");
    }

    public void OuttakeStop() {
        OuttakeFront.setPower(0);
        telemetry.addData("Outtake", "OFF");
    }

    private void ShootRing() {
        PushRing();
        waitForTime(300, "Shooting Ring");
        PushReset();
        waitForTime(1000, "Resetting shooter");
    }

    private void waitForTime(int mills, String caption) {
        telemetry.addData("W", caption);
        telemetry.update();
        // Wait until the shooter charges up
        sleep(mills);
    }

    private void DriveToLaunchLine() {
        ForwardUntilAtTargetPosition(70);
    }


    private void IntakeStop() {
    }

    private void IntakeStart() {
    }

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
        //waitForTime(3000, "Finished " + caption);
    }

//    private void moveWobbleBackward(int position) {
//
//        // Get Current position in ticks
//        wobbleCurrentPosition = GetWobblePosition();
//
//        position = position * -1;
//
//        // Get Target position by adding current position and # of ticks to travel
//        wobbleTargetPosition = wobbleCurrentPosition + calcDistance(position);
//
//        // Loop until we reach the target
//        while (leftCurrentPosition > leftTargetPosition) {
//
//            if (!opModeIsActive()) {
//                break;
//            }
//
//            telemetry.addData("W", "Inside While Loop...");
//
//            // Recalculate the current position
//            wobbleCurrentPosition = GetWobblePosition();
//
//            odometerWobble.setPower(0.5);
//
//            telemetry.addData("W", "Moving Wobbly Boi");
//            //LoadTelemetryData();
//            telemetry.update();
//        }
//
//        odometerWobble.setPower(0);
//
//    }

    private void LiftWobble() {
    }

//    private void GrabWobble() {}

    private void DropWobble() {
    }

    private void DeliverWobble(String ringStack) {
        if (ringStack == "Quad") {
            // Drive to Target Zone C
//            telemetry.addData("R", ringStack + 4);
            ForwardUntilAtTargetPosition(74.25);
        } else if (ringStack == "Single") {
            // Drive to Target Zone B
//            telemetry.addData("R", ringStack + 1);
            ForwardUntilAtTargetPosition(56.25);
            rotate(90, 0.5);
            ForwardUntilAtTargetPosition(24);
            rotate(-90, 0.5);
        } else {
            // Drive to Target Zone A
//            telemetry.addData("R", ringStack + 0);
            ForwardUntilAtTargetPosition(27.75);
        }
        telemetry.update();
        waitForTime(10000, "RingStack");
    }

    private void IdentifyRingNumber() {
    } // TODO: make int

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

        telemetry.clearAll();
        telemetry.update();

        // Loop until we reach the target
        while (leftCurrentPosition < leftTargetPosition && opModeIsActive()) {

//            if (!opModeIsActive()) {
//                break;
//            }

            //telemetry.addData("W", "Inside While Loop...");

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightForwards();

            //telemetry.addData("CL", "Current Left: " + GetLeftPosition());
            telemetry.addData("CL", "Current Left   : " + leftCurrentPosition);
            telemetry.addData("TP", "Target Position: " + leftTargetPosition);
            //IMUStuff();
            //telemetry.addData("CR", "Current Right: " + GetRightPosition());
            //telemetry.addData("CB", "Current Back: " + GetBackPosition());
            //telemetry.addData("CL", "Current Left", +leftCurrentPosition);
            //telemetry.addData("CR", "Current Right", +rightCurrentPosition);
            //telemetry.addData("TL", "Target Left", +leftTargetPosition);
            //telemetry.addData("TR", "Target Right", +rightTargetPosition);
            //telemetry.addData("LT", "Distance: " + calcDistance(distanceInch));
            //telemetry.addData("1 imu heading", lastAngles.firstAngle);
            //telemetry.addData("2 global heading", globalAngle);
            //telemetry.addData("correction", correction);
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
        return odometerLeft.getCurrentPosition() * -1;
    }

    private int GetRightPosition() {
        return odometerRight.getCurrentPosition() * -1;
    }

    private int GetBackPosition() {
        return odometerBack.getCurrentPosition();
    }

    private int GetWobblePosition() {
        return odometerWobble.getCurrentPosition();
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
        while (leftCurrentPosition > leftTargetPosition && opModeIsActive()) {

            telemetry.addData("W", "Inside While Loop...");

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightBackwards();

//            telemetry.addData("LT", "Left Target  : " + leftTargetPosition);
//            telemetry.addData("LT", "Left Current : " + leftCurrentPosition);
//            telemetry.addData("LT", "Distance: " + calcDistance(distanceInch));
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("correction", correction);
//            telemetry.addData("getP", pidDrive.getP());
//            telemetry.addData("getI", pidDrive.getI());
//            telemetry.addData("getD", pidDrive.getD());
//            telemetry.addData("getError", pidDrive.getError());
//            //LoadTelemetryData();
//            telemetry.update();
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
                ProMotorControl(-0.0, -power, 0.0);
            } else if (strafeDirection == StrafeDirection.Right) {
                ProMotorControl(-0.0, power, 0.0);
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
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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

//    private String ringCount() {
//        String label = "";
//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                telemetry.addData("# Object Detected", updatedRecognitions.size());
//                // step through the list of recognitions and display boundary info.
//                int i = 0;
//                for (Recognition recognition : updatedRecognitions) {
//                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                            recognition.getLeft(), recognition.getTop());
//                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                            recognition.getRight(), recognition.getBottom());
//                    label = recognition.getLabel();
//                }
//                telemetry.update();
//                return label;
//            }
//        }
//        return "";
//    }
//

    private void TurnOnCamera() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    private void TurnOffCamera() {
        if (tfod != null) {
            tfod.shutdown();
            vuforia.getCamera().close();
        }
    }

    private RingsFound FindRings() {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
//        Boolean allDone = false;
//
        RingsFound returnRings = RingsFound.None;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {

            if (tfod != null) {
//                telemetry.addData("TFOD", "Not null");
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
//                    telemetry.addData("Recognition", "Not null");
//                    telemetry.addData("Recognition Size", updatedRecognitions.size());
                    if (updatedRecognitions.size() > 0) {
                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData("Ring Label", recognition.getLabel());
//                        sleep(2000);

                            if (recognition.getLabel().equals("Quad")) {
                                returnRings = RingsFound.Quad;
                            } else if (recognition.getLabel().equals("Single")) {
                                returnRings = RingsFound.Single;
                            }
//                            else {
//                                returnRings = RingsFound.NotFound;
//                            }
                        }
                    }
//                    else {
//                        returnRings = RingsFound.None;
//                    }
                    //return RingsFound.None;
                }
//                else {
//                    telemetry.addData("Recognition", "NULL");
//                    returnRings =  RingsFound.None;
//                }
            }
//            else {
//                telemetry.addData("TFOD", "NULL");
//                returnRings = RingsFound.NotFound;
//            }
//            telemetry.addData("Timer", timer.milliseconds());
//            telemetry.update();
            if (timer.milliseconds() >= 1000)
                break;
            //sleep(2000);
        }

        return returnRings;
//        return null;
    }

}