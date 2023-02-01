package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.structure.Centric_Drive;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;

@TeleOp
public class First_Chasis_Tests extends LinearOpMode {


    ChasisInit cs = new ChasisInit();
    Centric_Drive CDrive = new Centric_Drive();



    @Override
    public void runOpMode() throws InterruptedException {



        CDrive.Init(hardwareMap,gamepad1);
        cs.init(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            CDrive.run();

            if(gamepad1.a){
                cs.BackRight.setPower(1);
                telemetry.addData("Back_Right Has been Pressed",cs.BackRight.getPower());
            }else {
                telemetry.addData("Back_Right Has been Released",cs.BackRight.getPower());
                cs.BackRight.setPower(0);
            }

            if(gamepad1.b){
                cs.BackLeft.setPower(1);
                telemetry.addData("Back_Left Has been Pressed",cs.BackLeft.getPower());
            }else {
                telemetry.addData("Back_Left Has been Released",cs.BackLeft.getPower());
                cs.BackLeft.setPower(0);
            }

            if(gamepad1.x){
                cs.FrontLeft.setPower(1);
                telemetry.addData("Front_Left Has been Pressed",cs.FrontLeft.getPower());
            }else {
                telemetry.addData("Front_Left Has been Released",cs.FrontLeft.getPower());
                cs.FrontLeft.setPower(0);
            }

            if(gamepad1.y){
                cs.FrontRight.setPower(1);
                    telemetry.addData("Front_Right Has been Pressed",cs.FrontRight.getPower());
            }else {
                telemetry.addData("Front_Right Has been Released",cs.FrontRight.getPower());
                cs.FrontRight.setPower(0);
            }

            telemetry.addData("Back_Right Port",cs.BackRight.getPortNumber());
            telemetry.addData("Back_Left Port",cs.BackLeft.getPortNumber());
            telemetry.addData("Front_Right Port",cs.FrontRight.getPortNumber());
            telemetry.addData("Front_Left Port",cs.FrontLeft.getPortNumber());
            telemetry.update();

        }

    }

}
