package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.PIDController;

import java.io.PrintWriter;
import java.io.StringWriter;

@Autonomous(name = "GyroTest", group = "Christian")
public class GyroTesting  extends LinearOpMode {

    // Motors
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    private DcMotor odometerLeft;

    // PID
    private double P = .0025, I = 0.02, D = 0.01;
    private PIDController pidDrive;

    // Gyroscope
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private Orientation lastAngles = new Orientation();

    private double globalAngle, power = 0.6, correction, rotation, MAX_POWER = power, MIN_POWER = power * -1;
    private double MIN_CORRECTION = 0.0, MAX_CORRECTION = 0.1;

    private final double COUNTS_PER_INCH = 290; //307.699557;

    private int leftCurrentPosition = 0;
    private double leftTargetPosition = 0;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();

        telemetry.addData("Waiting for start","");
        telemetry.update();
        waitForStart();


        ForwardUntilAtTargetPosition(70);

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


        odometerLeft = hardwareMap.dcMotor.get("Front Left");
        odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftCurrentPosition = odometerLeft.getCurrentPosition();

        // Initialize IMU
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        telemetry.addData("GC", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
        modernRoboticsI2cGyro.calibrate();

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

    }

    private void resetAngle() {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private void IMUStuff() {

        //telemetry.addData("p", power);
        //telemetry.addData("C", correction);
        telemetry.addData("a", getAngle());
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        //telemetry.addData("correction", correction);
        //telemetry.addData("getP", pidDrive.getP());
        //telemetry.addData("getI", pidDrive.getI());
        //telemetry.addData("getD", pidDrive.getD());
        //telemetry.addData("getError", pidDrive.getError());
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

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightForwards();

            telemetry.addData("CL", "Current Left   : " + leftCurrentPosition);
            telemetry.addData("TP", "Target Position: " + leftTargetPosition);
            telemetry.update();
        }
        BasicMotorControl(0.0);

    }

    private void SetPIDForward() {

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(MIN_CORRECTION, MAX_CORRECTION);
        pidDrive.setInputRange(-90, 90);
        pidDrive.setContinuous(true);
        pidDrive.enable();
    }

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

    private int GetLeftPosition() {
        return odometerLeft.getCurrentPosition() * -1;
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

}
