package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Concept: Shooter Testing 001", group = "Concept")
public class ShooterTesting001 extends OpMode {

    /** Declare Hardware **/
    private DcMotor IntakeLeft;
    private DcMotor IntakeRight;
    //hardwareMap.get(DcMotorEx.class, "Front Left");

    private double INTAKE_SPEED = 1;


    @Override
    public void init() {
        IntakeLeft = hardwareMap.dcMotor.get("Left Intake");
        IntakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeLeft.setPower(0);
        IntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeRight = hardwareMap.dcMotor.get("Right Intake");
        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeRight.setPower(0);
        IntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        boolean button_a = gamepad1.a;


        if (button_a) {
            IntakeIn();
        } else {
            IntakeStop();
        }


    }
    public void IntakeIn() {
        IntakeLeft.setPower(INTAKE_SPEED);
        IntakeRight.setPower(INTAKE_SPEED);
    }

    public void IntakeStop() {
        IntakeLeft.setPower(0);
        IntakeRight.setPower(0);
    }
}
