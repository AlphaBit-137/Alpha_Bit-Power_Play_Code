package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Program temporar
@Autonomous
public class TestDetection extends LinearOpMode {

    BlockDetection bloc = new BlockDetection();

    public int caz=1;

    @Override
    public void runOpMode() throws InterruptedException {

        bloc.init(hardwareMap);

        while (!isStarted()) {
            if (bloc.Right_percent <= 100 && bloc.Right_percent >=70) {
                telemetry.addData("Cazul", 3);
                caz = 3;
            }else if  (bloc.Right_percent <= 50 && bloc.Right_percent >=20) {
                telemetry.addData("Cazul", 2);
                caz = 2;
            }else {
                telemetry.addData("Cazul", 1);
                caz = 1;
            }

        }

        waitForStart();
        while(opModeIsActive()){
            //telemetry.addData("cazul",caz);
            telemetry.addData("RightPercent", bloc.Right_percent);
            telemetry.update();
        }

    }
}