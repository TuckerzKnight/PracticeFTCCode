/*
Base handles bulk reading, and hub voltage
 */

package org.firstinspires.ftc.teamcode;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class Base {

    VoltageSensor myControlHubVoltageSensor;
    public double presentVoltage;


    public static double powerMultiplier;

    public static int voltageHistoryLength = 5;
    public static double voltageHistory[] = new double[voltageHistoryLength]; //store the last few values of control hub voltage
    double averageVoltage;
    int cycleNum = 0;
    public void voltageInit(HardwareMap hardwareMap){
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void init (HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        voltageInit(hardwareMap);
    }
    public void update (Telemetry telemetry) {
        presentVoltage = myControlHubVoltageSensor.getVoltage();

        //get the average voltage of the past 5 cycles
        averageVoltage = 0;
        for (int i = 0; i < voltageHistoryLength; i++) {
            averageVoltage += voltageHistory[i];
        }
        averageVoltage /= voltageHistoryLength;
        powerMultiplier = min(presentVoltage / averageVoltage, 1);
        voltageHistory[cycleNum%voltageHistoryLength] = presentVoltage;
        cycleNum++;
        telemetry.addData("voltage",presentVoltage);
        telemetry.addData("powerMultiplier",powerMultiplier);


    }
}
