package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.comp.PIDController;

@Disabled
@TeleOp(name = "Arm Test", group = "Testing")
public class ArmTest extends OpMode {

    //wobbly boi motor
    private DcMotor WobbleGrabber;
    private DcMotor odometerWobble;

    private double P = .0025, I = 0.02, D = 0.01;
    private PIDController pidDrive;

    private double globalAngle, power = 0.4, correction, rotation, MAX_POWER = power, MIN_POWER = power * -1;
    private double MIN_CORRECTION = 0.2, MAX_CORRECTION = 0.3;
    private int wobbleCurrentPosition = 0;

    @Override
    public void init() {


        //initialize wobbly boi
//        WobbleGrabber
        WobbleGrabber = hardwareMap.dcMotor.get("Wobble Grabber");
        WobbleGrabber.setDirection(DcMotorSimple.Direction.FORWARD);
        WobbleGrabber.setPower(0);
        WobbleGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometerWobble = hardwareMap.dcMotor.get("Wobble Grabber");
        odometerWobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerWobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleCurrentPosition = getPosition();


        /* Set PID proportional value to produce non-zero correction value when robot veers off
         * straight line. P value controls how sensitive the correction is. */
        pidDrive = new PIDController(P, I, D);

        SetPIDForward(getPosition());
    }

    boolean firstClickHold = true;
    boolean isHolding = false;

    @Override
    public void loop() {
        boolean one_button_a = gamepad1.a;


        // Reverse Mode controls
        if (one_button_a && firstClickHold){
            isHolding = !isHolding;
            firstClickHold = false;
            SetPIDForward(getPosition());
        }

        if (!one_button_a){
            firstClickHold = true;
        }

        // Tell if Reverse Mode is engaged
        if (isHolding) {

            HoldArmInPlace();

            telemetry.addData("REVERSE MODE", "ENGAGED");
        } else {
            telemetry.addData("REVERSE MODE", "DISENGAGED");
        }
    }

    private void HoldArmInPlace() {
        correction = pidDrive.performPID(getPosition());

        if (power - correction <= 0.2) {
            power += correction;
        }

        telemetry.addData("LT", "Power Sub    : " + (power + correction));

        WobbleGrabber.setPower(power + correction);
    }

    private int getPosition() {
        return odometerWobble.getCurrentPosition();
    }


    private void SetPIDForward(int setPoint) {

        // Set up parameters for driving in a straight line.
        pidDrive.setInputRange(-90000, 90000);
        pidDrive.setSetpoint(setPoint);
        pidDrive.setOutputRange(MIN_CORRECTION, MAX_CORRECTION);
        pidDrive.setContinuous(true);
        pidDrive.enable();
    }

}
