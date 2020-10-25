package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Concept: Shooter Testing 002", group = "Concept")
public class ShooterTesting002 extends OpMode {

    /** Declare Hardware **/
    private DcMotor OuttakeFront;
    private DcMotor OuttakeBack;
    //hardwareMap.get(DcMotorEx.class, "Front Left");

    private double OuttakeFrontPower = 0.2;
    private double OuttakeBackPower = 1;
    private int k = 0;
    private int j = 0;


    @Override
    public void init() {
        OuttakeFront = hardwareMap.dcMotor.get("Front Outtake");
        OuttakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeFront.setPower(0);
        OuttakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeBack = hardwareMap.dcMotor.get("Back Outtake");
        OuttakeBack.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeBack.setPower(0);
        OuttakeBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        boolean button_a = gamepad1.a;
        boolean button_b = gamepad1.b;

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
            } else if (k < 1 && j == 4) {   // 6-% power
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

        if (button_a) {
            IntakeIn();
        } else {
            IntakeStop();
        }

        telemetry.addData("Power", OuttakeFrontPower * 100 + "%");


    }

    /** Functions **/
    public void IntakeIn() {
        OuttakeFront.setPower(OuttakeFrontPower);
        OuttakeBack.setPower(OuttakeBackPower);
        telemetry.addData("Outtake", "ON");
    }

    public void IntakeStop() {
        OuttakeFront.setPower(0);
        OuttakeBack.setPower(0);
        telemetry.addData("Outtake", "OFF");
    }
}
