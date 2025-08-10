package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;
public class Slider {
    private Robot robot;
    private Gamepad gamepad;
    private double manual_speed_factor;
    static final private double auto_power = 0;
    static final private int pos_max = 0;
    static final private int pos_min = 0;

    private boolean checkLimitSwitch = false;


    public Slider(Robot robot)
    {
//        super(robot, robot.motorSlider, pos_min, pos_max, auto_power, true);
        this.robot = robot;
    }

    public void setPower(double power) {
        robot.motorSlider.setPower(power);
    }
    public int getCurrentPosition() {
        return robot.motorSlider.getCurrentPosition();
    }

}
