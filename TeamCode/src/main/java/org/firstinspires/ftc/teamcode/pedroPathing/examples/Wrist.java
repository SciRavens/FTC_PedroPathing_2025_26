package org.firstinspires.ftc.teamcode.pedroPathing.examples;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.Gamepad;


public class Wrist {
    private Robot robot;
    private double target;
    private double speed = 0.05;

    private boolean speed_control = false;
    private double cur_pos = 0.0;

    private double max_speed = 0.2; //0.1
    private double threshold = 0.005;
    private final double P = 0.01;

    public Wrist(Robot robot) {

        this.robot = robot;
        this.target = robot.servoWrist.getPosition();
        cur_pos = target;

    }

    private void setSCTarget(double target) {
        speed_control = true;
        this.target = target;
    }
    private void setPos(double pos, boolean sc_on)
    {
        if (sc_on) {
            setSCTarget(pos);
        } else {
            robot.servoWrist.setPosition(pos);
            cur_pos = pos;
        }
    }
}
