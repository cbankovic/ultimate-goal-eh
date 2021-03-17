package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import  org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Double voltage = 0.0;
    Double OuttakeFrontPower = 0.535;
    double OuttakePower = 0.0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    // Battery voltage shows 13.00 while not shooting
    // While shooting it shows 12.9
    public void AdjustOuttakeSpeed() {
        voltage = getBatteryVoltage();

        if (voltage >= 14) {
            OuttakePower = 0.49;
        } else if (voltage >= 13.5) {
            OuttakePower = 0.50;
        } else if (voltage >= 13.0) {
            OuttakePower = 0.52125;
        } else if (voltage >= 12.8) {
            OuttakePower = OuttakeFrontPower;
        } else if (voltage >= 12.5) {
            OuttakePower = 0.56;
        } else if (voltage > 0) {
            OuttakePower = 0.58;
        } else {
            OuttakePower = OuttakeFrontPower;
        }

        telemetry.addData("Voltage", voltage);
        telemetry.addData("OuttakePower", OuttakePower);
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
