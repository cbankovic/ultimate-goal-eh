package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.PIDController;

public class drive {


    private DcMotorEx[] motors = new DcMotorEx[4];
    public boolean slowMode = false;
    private double PERCENT_TO_SLOW = 0.5;
    private boolean opModeIsActive = false;


    // Encoders
    private DcMotor EncVerticalLeft;
    private int leftCurrentPosition = 0;
    private double leftTargetPosition = 0;
    private final double COUNTS_PER_INCH = 290; //307.699557;

    // PID
    private double P = .0025, I = 0.0, D = 1.0;
    private PIDController pidRotate, pidDrive;
    private double globalAngle, power = 0.4, correction, rotation, MAX_POWER = power, MIN_POWER = power * -1;
    private double MIN_CORRECTION = 0.0, MAX_CORRECTION = 0.1;

    // Gyroscope
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private Orientation lastAngles = new Orientation();

    // Intake
    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;
    private double INTAKE_SPEED = 0.3;

    public enum StrafeDirection {
        Right,
        Left
    }

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    public drive(HardwareMap hardwareMap, boolean inTeleop) {

        motors[0] = hardwareMap.get(DcMotorEx.class, "Front Left");
        motors[1] = hardwareMap.get(DcMotorEx.class, "Front Right");
        motors[2] = hardwareMap.get(DcMotorEx.class, "Back Left");
        motors[3] = hardwareMap.get(DcMotorEx.class, "Back Right");

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
        if (!inTeleop) {
            for (int i = 0; i < 4; i++) {
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        } else {
            for (int i = 0; i < 4; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        // Intake
        IntakeLeft = hardwareMap.dcMotor.get("Left Intake");
        IntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeLeft.setPower(0);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeRight = hardwareMap.dcMotor.get("Right Intake");
        IntakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeRight.setPower(0);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Encoders
        EncVerticalLeft = hardwareMap.dcMotor.get("Front Left");
        EncVerticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncVerticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        modernRoboticsI2cGyro.calibrate();

        /* Set PID proportional value to start reducing power at about 50 degrees of rotation.
         * P by itself may stall before turn completed so we add a bit of I (integral) which
         * causes the PID controller to gently increase power if the turn is not completed. */
        pidRotate = new PIDController(.003, .00003, 0);

        /* Set PID proportional value to produce non-zero correction value when robot veers off
         * straight line. P value controls how sensitive the correction is. */
        pidDrive = new PIDController(P, I, D);

        resetAngle();

        GetGyroInfo();

        SetPIDForward();
    }

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    public void ProMotorControl(float one_right_stick_y, float one_right_stick_x, float one_left_stick_x) {
        float powerRightY = one_right_stick_y;
        float powerRightX = one_right_stick_x;
        float powerLeftX = one_left_stick_x;

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

        motors[0].setPower(v1);
        motors[1].setPower(v2);
        motors[2].setPower(v3);
        motors[3].setPower(v4);
    }

    public void IntakeIn() {
        IntakeLeft.setPower(INTAKE_SPEED);
        IntakeRight.setPower(INTAKE_SPEED);
    }

    public void IntakeOut() {
        IntakeLeft.setPower(INTAKE_SPEED);
        IntakeRight.setPower(INTAKE_SPEED);
    }

    public void IntakeStop() {
        IntakeLeft.setPower(0);
        IntakeRight.setPower(0);
    }

    public void DriveStraightForwards() {
        correction = pidDrive.performPID(getAngle());

        if (power - correction <= 0.2) {
            power += correction;
        }

        motors[0].setPower(power - correction);
        motors[1].setPower(power + correction);
        motors[2].setPower(power - correction);
        motors[3].setPower(power + correction);
    }

    public void DriveStraightBackwards() {
        correction = pidDrive.performPID(getAngle());

        if (power + correction >= -0.2) {
            power -= correction;
        }

        motors[0].setPower(power - correction);
        motors[1].setPower(power + correction);
        motors[2].setPower(power - correction);
        motors[3].setPower(power + correction);
    }

    public void ForwardUntilAtTargetPosition(double distanceInch) {

        power = MAX_POWER;

        SetPIDForward();

        DriveStraightForwards();

        // Get Current position in ticks
        leftCurrentPosition = GetLeftPosition();

        // Get Target position by adding current position and # of ticks to travel
        leftTargetPosition = leftCurrentPosition + calcDistance(distanceInch);

        // Loop until we reach the target
        while (leftCurrentPosition < leftTargetPosition) {

            if (!opModeIsActive) {
                break;
            }

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightForwards();

        }
        BasicMotorControl(0.0);

    }

    public void BackwardUntilAtTargetPosition(double distanceInch) {
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

            if (!opModeIsActive) {
                break;
            }

            // Recalculate the current position
            leftCurrentPosition = GetLeftPosition();

            DriveStraightBackwards();
        }
        BasicMotorControl(0.0);

    }

    public void StrafeUntilTimerReached(StrafeDirection strafeDirection, float power, int mills) {

        timer.reset();

        // Loop until we reach the target
        while (timer.milliseconds() < mills) {

            if (!opModeIsActive) {
                break;
            }
            if (strafeDirection == StrafeDirection.Left) {
                ProMotorControl(-0.1f, -power, 0.0f);
            } else if (strafeDirection == StrafeDirection.Right) {
                ProMotorControl(-0.1f, power, 0.0f);
            }
        }

        ProMotorControl(0.0f, 0.0f, 0.0f);
    }

    public void ForwardUntilTimerReached(int mills) {

        timer.reset();

        // Loop until we reach the target
        while (timer.milliseconds() < mills) {

            if (!opModeIsActive) {
                break;
            }

            // Call BasicMotorControl with new speed
            BasicMotorControl(0.5);
        }

        BasicMotorControl(0.0);

    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {
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
        pidRotate.setInputRange(0, degrees);
        pidRotate.setSetpoint(degrees);
        pidRotate.setOutputRange(.5, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        /* getAngle() returns + when rotating counter clockwise (left) and - when rotating
         * clockwise (right). */

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive && getAngle() == 0) {
                motors[0].setPower(power);
                motors[1].setPower(power);
                motors[2].setPower(-power);
                motors[3].setPower(-power);
                Sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                motors[0].setPower(-power);
                motors[1].setPower(-power);
                motors[2].setPower(power);
                motors[3].setPower(power);
            } while (opModeIsActive && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                motors[0].setPower(-power);
                motors[1].setPower(-power);
                motors[2].setPower(power);
                motors[3].setPower(power);
            } while (opModeIsActive && !pidRotate.onTarget());

        // turn the motors off.
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

        rotation = getAngle();

        // reset angle tracking on new heading.
        resetAngle();
    }

    //******************************************************************
    // Get the inputs from the controller for power [ BASIC ]
    //******************************************************************
    public void BasicMotorControl(double right_stick_y) {
        motors[0].setPower(right_stick_y);
        motors[1].setPower(right_stick_y);
        motors[2].setPower(right_stick_y);
        motors[3].setPower(right_stick_y);
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

    private int GetLeftPosition() {
        return EncVerticalLeft.getCurrentPosition() * -1;
    }

    private double calcDistance(double distanceInch) {
        return COUNTS_PER_INCH * distanceInch;
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


    public void resetAngle() {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public boolean isCalibrating() {
        return modernRoboticsI2cGyro.isCalibrating();
    }

    public boolean isOpModeIsActive() {
        return opModeIsActive;
    }

    public void setOpModeIsActive(boolean opModeIsActive) {
        this.opModeIsActive = opModeIsActive;
    }

    public void Sleep(int mills) {


        try {
            Thread.sleep(mills);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
