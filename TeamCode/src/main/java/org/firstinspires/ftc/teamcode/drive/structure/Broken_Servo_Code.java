package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Broken_Servo_Code {

    public CRServo servo;
    Gamepad servo_gamepad;
    public boolean Up = false;
    public boolean down = false;

    public boolean decision = false;
    public ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwmap, Gamepad servo_gamepad) {
        servo = hwmap.get(CRServo.class, "Servo1");
        this.servo_gamepad = servo_gamepad;
        servo.setPower(0);
    }

    public void initTimer() {
        timer.reset();
    }

    public void update() {
        if (servo_gamepad.right_bumper) {
            if (decision && !Up) {
                Up = true;
                initTimer();
            }
            decision = false;
        } else decision = true;

        if (Up) {

            if (timer.seconds() < 0.5) servo.setPower(1);
            else {
                Up = false;
                servo.setPower(0);
            }
        }
    }
}
