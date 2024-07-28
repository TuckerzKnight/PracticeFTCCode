package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class Main {

    private FtcDashboard dashboard;
    public void init (HardwareMap hMap) {
        List<LynxModule> allHubs = hMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        dashboard = FtcDashboard.getInstance();
    }
    public void update (Telemetry telemetry) {

        telemetry.addData("heading", Drivebase.robotStats[0]);
        telemetry.addData("direction", Drivebase.robotStats[1]);
        telemetry.addData("desiredHeading", Drivebase.desiredHeading);
        telemetry.addData("fixedHeading", Drivebase.fixedHeading);
        telemetry.addData("timesAcrossZero", Drivebase.timesAcrossZero);
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("telemetry", telemetry);
        packet.fieldOverlay()
                .setFill("blue")
                .fillRect((Kinematics.X-Kinematics.robotwidth),(Kinematics.Y-Kinematics.robotwidth),40,40)
                .strokeCircle(100,100,5);
        dashboard.sendTelemetryPacket(packet);
    }
}
