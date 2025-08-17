package org.firstinspires.ftc.teamcode.pedroPathing.TeamCode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.TeamCode.Robot;

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

//    public void InitialPose() {autoOp(robot.slider_Initial_Pose_ticks); checkLimitSwitch = true;}
//    public void LowBasket() {autoOp(robot.slider_LowBasket_ticks);}
//    //    Move slider height to Low basket
//    public void LowChamber() {autoOp(robot.slider_LowChamber_ticks);}
//    //    Move slider height to Low Chamber
//    public void HighBasket() {autoOp(robot.slider_HighBasket_ticks);}
//    //    Move slider height to Low basket
//    public void HighChamber() {
//        autoOp(robot.slider_HighChamber_ticks);
//    }
//    public void HighChamberBack() {
//        autoOp(robot.slider_HighChamberBack_ticks);
//    }

}
