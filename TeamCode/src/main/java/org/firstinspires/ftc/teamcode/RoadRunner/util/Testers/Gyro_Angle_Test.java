package org.firstinspires.ftc.teamcode.RoadRunner.util.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Gyroscope;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;

public class Gyro_Angle_Test extends LinearOpMode {


        ChasisInit CSI = new ChasisInit();
        private ElapsedTime Time = new ElapsedTime();
        private Orientation Last_Angles = new Orientation();
        private double Current_Angle = 0.0;
        Gyroscope gyros = new Gyroscope();

        @Override
        public void runOpMode() throws InterruptedException {

            CSI.init(hardwareMap);
            gyros.Init(hardwareMap);

            waitForStart();

            Turn(90);

            sleep(10000);

            TurnTo(-90);

        }

        public void Reset_Angle(){

            Last_Angles = gyros.AngularOrientation();
            Current_Angle = 0;

        }

        public double Get_Angle(){

            Orientation orientation = gyros.AngularOrientation();

            double Delta_Angle = orientation.firstAngle - Last_Angles.firstAngle;

            if(Delta_Angle > 180){
                Delta_Angle -= 360;
            }else if(Delta_Angle <= -180){
                Delta_Angle += 360;
            }

            Current_Angle += Delta_Angle;
            Last_Angles = orientation;

            telemetry.addData("Gyro",orientation.firstAngle);

            return Current_Angle;


        }

        public void Turn(double degrees){

            Reset_Angle();

            double error = degrees;

            while(opModeIsActive() && Math.abs(error) > 2){

                double Motor_Power = (error < 0 ? -0.3 : 0.3);

                CSI.setMotortPower(-Motor_Power,Motor_Power,-Motor_Power,Motor_Power);


                error = degrees - Get_Angle();

                telemetry.addData("error" , error);
                telemetry.update();
            }

            CSI.setMotortPower(0,0,0,0);

        }

        public void TurnTo(double degrees){

            Orientation orientation = gyros.AngularOrientation();

            double error = degrees - orientation.firstAngle;

            if(error > 180){
                error -= 360;
            } else if(error < -180){
                error += 360;
            }

            Turn(error);

        }

}
