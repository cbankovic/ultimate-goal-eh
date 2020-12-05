package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.PIDController;

import java.io.PrintWriter;
import java.io.StringWriter;

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

    // Shooter Motors
    private DcMotor OuttakeFront;
    private Servo OuttakeBack;
    private double OuttakeFrontPower = 0.2;
    private double OuttakeBackPower = 1;

    // Encoders
    private DcMotor odometerLeft;
    private DcMotor odometerRight;
    private DcMotor odometerBack;

    private int leftCurrentPosition = 0;
    private double leftTargetPosition = 0;

    private final double COUNTS_PER_INCH = 290; //307.699557;

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

                // Do stuff

            } catch (Exception ex) {

                LoadExceptionData(ex, "while");

            } finally {

                telemetry.update();
            }
            break;
        }

    }


    /***************
     *** METHODS ***
     ***************/


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

//        OuttakeBack = hardwareMap.dcMotor.get("Back Outtake");
//        OuttakeBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        OuttakeBack.setPower(0);
//        OuttakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        OuttakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        OuttakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Encoders
        odometerLeft = hardwareMap.dcMotor.get("Front Left");

        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftCurrentPosition = odometerLeft.getCurrentPosition();

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

        telemetry.addData("Status", "Initialized :D");
        telemetry.update();

    }

    // TODO: Code These Functions
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

    private void ShootRing() {
        // Start Shooter
        // Stop Shooter
    }

    private void IntakeStop() {}

    private void IntakeStart() {}

    private void CollectRing() {
        // Start Intake
        // Stop Intake
    }

    private void LiftWobble() {}

    private void GrabWobble() {}

    private void DropWobble() {}

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

    private int GetLeftPosition() {
        return odometerLeft.getCurrentPosition() * -1;
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


}