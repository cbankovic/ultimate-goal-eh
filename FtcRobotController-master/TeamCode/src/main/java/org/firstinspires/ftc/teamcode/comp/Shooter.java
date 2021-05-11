package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class Shooter {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Double voltage = 0.0;
    Double OuttakeFrontPower = 0.535;
    public double OuttakePower = 0.0;
    double previousPower = 0.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // Battery voltage shows 13.00 while not shooting
    // While shooting it shows 12.9
    public void AdjustOuttakeSpeed(double powerShotMultiplier, boolean isAuto) {
        //if (ForcePower == 0) {

//        if (powerShotMultiplier == 0.0)
//            powerShotMultiplier = 1.0;

        voltage = getBatteryVoltage();

        if (voltage >= 14) {
            OuttakePower = 0.49000;
        } else if (voltage >= 13.7) {
            OuttakePower = 0.50000;
        } else if (voltage >= 13.5) {
            OuttakePower = 0.50050;
        } else if (voltage >= 13.4) {
            OuttakePower = 0.50075;
        } else if (voltage >= 13.3) {
            OuttakePower = 0.50125;
        } else if (voltage >= 13.2) {
            OuttakePower = 0.50400;
        } else if (voltage >= 13.15) {
            OuttakePower = 0.52250;
        } else if (voltage >= 13.0) {
            OuttakePower = 0.52200;
        } else if (voltage >= 12.8) {
            OuttakePower = OuttakeFrontPower;
        } else if (voltage >= 12.5) {
            OuttakePower = 0.54000;
        } else if (voltage > 0) {
            OuttakePower = 0.55000;
        } else {
            OuttakePower = OuttakeFrontPower;
        }

        OuttakePower = OuttakePower * powerShotMultiplier;
//        } else {
//            OuttakePower = ForcePower;
//        }

        if (previousPower != OuttakePower && isAuto) {
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        previousPower = OuttakePower;
        telemetry.addData("Voltage", voltage);
        telemetry.addData("OuttakePower", OuttakePower);
        telemetry.update();
    }

    // https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTelemetry.java
    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
