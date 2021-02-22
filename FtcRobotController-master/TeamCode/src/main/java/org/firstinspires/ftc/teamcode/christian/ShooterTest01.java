package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Shooter Test 01", group = "Christian")
public class ShooterTest01 extends LinearOpMode {

    private DcMotor OuttakeFront;
    private Servo pusher;
    private double PUSHER_IN = 0.4;
    private double PUSHER_OUT = 0;
    private double OuttakeFrontPower = 0.675;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize pusher
        pusher = hardwareMap.get(Servo.class, "Pusher");
        pusher.setDirection(Servo.Direction.FORWARD);
        pusher.setPosition(PUSHER_IN);

        // Initialize Shooter
        OuttakeFront = hardwareMap.dcMotor.get("Front Outtake");
        OuttakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeFront.setPower(0);
        OuttakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            StartShooter();
            ShootRing();
            ShootRing();
            ShootRing();
            OuttakeStop();
            break;
        }
    }

    private  void StartShooter(){
        // Start Shooter
        OuttakeOn();
        waitForTime(5000, "Charging Shooter...");
    }

    private void ShootRing() {
        PushRing();
        waitForTime(300, "Shooting Ring");
        PushReset();
        waitForTime(1000, "Resetting shooter");
    }


    public void PushRing() {
        pusher.setPosition(PUSHER_OUT);
    }


    public void PushReset() {
        pusher.setPosition(PUSHER_IN);
    }

    private void waitForTime(int mills, String caption) {
        telemetry.addData("W", caption);
        telemetry.update();
        // Wait until the shooter charges up
        sleep(mills);
    }

    public void OuttakeOn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        telemetry.addData("Outtake", "ON");
    }

    public void OuttakeStop() {
        OuttakeFront.setPower(0);
        telemetry.addData("Outtake", "OFF");
    }

}
