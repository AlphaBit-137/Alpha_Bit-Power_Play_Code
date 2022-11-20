package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;
import org.firstinspires.ftc.teamcode.drive.structure.ServoClaw;
import org.firstinspires.ftc.teamcode.drive.structure.Slider;

@TeleOp
public class First extends LinearOpMode {

    //Pozitii Hipotetice
    final int max = 3250;
    final int min = -3250;

    public int poz = 2;

    public double Limit = 0.3;

    public double Rotate_Right;
    public double Rotate_Left;

    public boolean Chose;
    public boolean Chose2;

    public boolean turns;

    public double Power_Front = 0.5;

    public ChasisInit chasis = new ChasisInit();
    public Slider slider = new Slider();
    public Arm arm = new Arm();
    public ServoClaw claw = new ServoClaw();

    @Override
    public void runOpMode() throws InterruptedException {

        chasis.init(hardwareMap);
        slider.init(hardwareMap);
        arm.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            //Initiallizam variabilele
            double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

            Rotate_Right = Range.clip(gamepad1.right_trigger,0,Limit);
            Rotate_Left = Range.clip(gamepad1.left_trigger, 0, Limit);

            if (Rotation() == 2) {
                Turn = Rotate_Right;
            }
            else if(Rotation() ==1) {
                Turn = -Rotate_Left;
            }
            else {
                Turn = Range.clip(gamepad1.left_stick_x, -Limit, Limit);
            }


            if(Front() == 1) {
                Front = Power_Front;
            }
            else if(Front() == 2) {
                Front = -Power_Front;
            }
            else {
                Front = Range.clip(gamepad1.right_stick_y, -Limit, Limit);
            }


            Side = Range.clip(gamepad1.left_stick_y, -Limit, Limit);

            //Speed Modes
       /*     if(gamepad1.right_bumper) {
                if(Chose)Limit=Limit+0.1;
                if(Limit>1)Limit=1;
                Chose = false;
            } else Chose=true;

            if(gamepad1.left_bumper) {
                if(Chose2)Limit=Limit-0.1;
                if(Limit<0.1)Limit=0.1;
                Chose2 = false;
            } else Chose2=true; */

            //Assist pentru viteze treptat
            if(gamepad1.right_bumper){
                Sped(1);
            }else Chose2 = true;

            if(gamepad1.left_bumper){
                Sped(-1);
            }else Chose = true;

            //Asist pt viteze direct
            if(gamepad1.dpad_up){
                Limit = 1;
            }else if(gamepad1.dpad_right){
                Limit = 0.6;
            }else if(gamepad1.dpad_down){
                Limit = 0.3;
            }

            //Calcularea puterii redate motoarelor
            Sum = Range.clip(Front + Side, -1.0, 1.0);
            Diff = Range.clip(Front - Side, -1.0, 1.0);

            Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
            Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
            Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
            Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);

            //Ridicarea sliderului manual sau daca motorul este setat sa se duca undeva
            if((gamepad2.right_bumper && Check()!=1) || slider.SliderBUSY()){
               slider.switchToSliderUp();
            }else {
                if (gamepad2.left_bumper && Check()!=2) {
                    slider.switchToSliderDown();
                } else {
                    slider.switchToSliderSTOP();
                }
            }

            //Ridicarea bratului manual sau daca motorul este setat sa se duca undeva
            if((gamepad2.right_trigger!=0 && gamepad2.left_trigger==0) || arm.ArmBUSY()){
                arm.switchToArmUp();
            }else{
                if(gamepad2.right_trigger==0 && gamepad2.left_trigger!=0) {
                    arm.switchToArmDown();
                }else {
                    arm.switchToArmSTOP();
                }
            }

          //Prinderea conului folosind clestele
            if(gamepad2.y){
               SwitchClaw();
            }else turns=true;


            //Updatarea tuturor functiolor.
            MS(Drive1, Drive2, Drive3, Drive4);

            slider.update();
            arm.update();

            telemetry.update();
        }
    }

    void MS(double x1, double x2, double x3, double x4){
        chasis.BackLeft.setPower(x1);
        chasis.FrontRight.setPower(x2);
        chasis.FrontLeft.setPower(x3);
        chasis.BackRight.setPower(x4);

    }

    //Functia ce verifica daca sliderul a ajuns la min/max
    public int Check(){
        if(slider.slider.getCurrentPosition() > max)
        return 1;
        else if(slider.slider.getCurrentPosition() < min) return 2;
        else return 3;
    }

    //Functie pentru modificarea vitezelor
    void Sped(int i){
        if(i > 0) {
            if (Chose && Limit > 0.3) Limit += i;
            Chose = false;
        }else if(i < 0){
            if(Chose2 && Limit < 1)Limit += i;
            Chose2 = false;
        }
    }

    //Functie doar pt cleste, nu intreba de ce am fcto doar pt cleste
    void SwitchClaw(){
        if(turns){
            if(poz == 2){
                claw.Closed();
                poz=1;
            }else{
                claw.Open();
                poz=2;
            }
        }
        turns=false;
    }

    public int Rotation(){
        if(gamepad1.left_trigger !=0 && gamepad1.right_trigger==0) return 1;
        else if(gamepad1.right_trigger !=0 && gamepad1.left_trigger==0) return 2;
        else return 3;
    }

    public int Front() {
        if(gamepad1.dpad_up && !gamepad1.dpad_down) return 1;
        else if(gamepad1.dpad_down && !gamepad1.dpad_up) return 2;
        else return 3;
    }


}
