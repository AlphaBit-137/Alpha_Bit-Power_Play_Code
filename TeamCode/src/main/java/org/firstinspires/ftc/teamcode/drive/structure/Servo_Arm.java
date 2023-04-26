package org.firstinspires.ftc.teamcode.drive.structure;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Servo_Arm {

    public Servo s1;
    public Servo s2;

    Gamepad gamepad;

    //0.3 - normal, 1 - pole
    public static double Positions = 0.3;

    public void init(HardwareMap ahwMap,Gamepad gamepad) {
        this.gamepad = gamepad;
        s1 = ahwMap.get(Servo.class, "Arm_Servo1");
        s2 = ahwMap.get(Servo.class, "Arm_Servo2");

        goToPosition(Positions);
    }

    public void goToPosition(double move) {
        s1.setPosition(move);
        s2.setPosition(1-move);
    }

    public void run()
    {
        if(gamepad.dpad_up)
        {
            goToPosition(1);
        }

        if(gamepad.dpad_down)
        {
            goToPosition(0.3);
        }
    }

}
