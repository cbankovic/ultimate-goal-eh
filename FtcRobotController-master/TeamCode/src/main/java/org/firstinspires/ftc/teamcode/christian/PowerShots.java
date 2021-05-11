package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.comp.PIDController;
import org.firstinspires.ftc.teamcode.comp.Shooter;

import java.io.BufferedReader;

@Disabled
@Autonomous(name = "Powershot", group = "Test")
public class PowerShots extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {

        Initialize();

        telemetry.addData("S", "Ready to start");
        telemetry.update();

        waitForStart();

        StartShooter();

        sleep(4000);


        ShootRing();
        ShootRing();
        ShootRing();

        OuttakeStop();

    }

    public void OuttakeStop() {
        OuttakeFront.setPower(0);
        //telemetry.addData("Outtake", "OFF");
    }

    private void StartShooter() {

            // Start Shooter
            OuttakeOn();
    }
    public void OuttakeOn() {
        AdjustOuttakeSpeed(powerShotMultiplier);
    }


    private void Initialize() {


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

        shooter = new Shooter(hardwareMap, telemetry);

    }

    private void ShootRing() {
        AdjustOuttakeSpeed(powerShotMultiplier);
        PushRing();
        waitForTime(500, "Shooting Ring");
        PushReset();
        waitForTime(1500, "Resetting shooter");
    }

    double OuttakePower;

    private void AdjustOuttakeSpeed(double powerShotMultiplier) {
        shooter.AdjustOuttakeSpeed(powerShotMultiplier, true);
        SetOuttakePower(shooter.OuttakePower);
        telemetry.addData("Voltage", voltage);
        telemetry.addData("OuttakePower", OuttakePower);
    }

    private void SetOuttakePower(double power) {
        OuttakeFront.setPower(power);
    }


    public void PushRing() {
        pusher.setPosition(PUSHER_OUT);
    }

    public void PushReset() {
        pusher.setPosition(PUSHER_IN);
    }


    private void waitForTime(int mills, String caption) {
//        telemetry.addData("W", caption);
//        telemetry.update();
        // Wait until the shooter charges up
        sleep(mills);
    }


}
