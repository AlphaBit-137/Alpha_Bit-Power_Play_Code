package org.firstinspires.ftc.teamcode.drive.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.NoPermaMP;

/*
*
*Experimental class used to calculate the power returned to the motor using PID
*
 */

@Config
@TeleOp
public class Pid_Motor extends LinearOpMode {

    public DcMotorEx TestMotor;

    ElapsedTime timer = new ElapsedTime();
    MotionProfile MP = new MotionProfile();
    NoPermaMP nmp = new NoPermaMP();

   private double LastError = 0;
   private double IntegralSum = 0;

   public static double Kp = 0.035;
   public static double Ki = 0.0;
   public static double Kd = 0.0;

   public static double maxAccel = 100;
   public static double maxVelocity = 100;

   public double encoder_direction = 1;

   public static int targetPosition = 500;

   public double getEror;

   double time;

   private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet =new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        TestMotor = hardwareMap.get(DcMotorEx.class,"Arm");

        TestMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TestMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TestMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        TestMotor.setDirection(DcMotorEx.Direction.REVERSE);

        encoder_direction = GetEncoderDirection(targetPosition);

        telemetry.addData("Encoder_Direction",encoder_direction);

        double reference = targetPosition * encoder_direction;

        waitForStart();

        while (opModeIsActive()) {

            double state = TestMotor.getCurrentPosition() * encoder_direction;
            double power = encoder_direction * returnPower(reference, state);

            packet.put("encoderDirection", encoder_direction);
            packet.put("power", power);
            packet.put("position", state);
            packet.put("error", LastError);
            packet.put("firstR",nmp.firstReturn);
            packet.put("secondR",nmp.secondReturn);
            packet.put("thirdR",nmp.thirdReturn);
            packet.put("maxvel",nmp.max_vel);
            packet.put("acc_dt", nmp.acc_dt);
            packet.put("calc",nmp.calc);

            TestMotor.setPower(-power);

         telemetry.update();
          dashboard.sendTelemetryPacket(packet);
        }
        }

        public double returnPower(double reference, double state){

        double error = reference - state;

        double velocity = TestMotor.getVelocity();

        if(velocity == 0)velocity = 1;

        double InstantErrror = nmp.motion_profile(maxAccel,maxVelocity,error,error/velocity);

        time = error/velocity;

        getEror = InstantErrror;

        if(state == reference || state > reference)IntegralSum = 0;
        else{IntegralSum += InstantErrror * timer.seconds();}


        double derivative = (InstantErrror - LastError) / timer.seconds();

        double outpput = (InstantErrror * Kp) + (derivative * Kd) + (IntegralSum * Ki);


        timer.reset();
        LastError = error;

        return outpput;
        }

        public double GetEncoderDirection(int TargetPosition)
        {
            if(TargetPosition < 0){
                return -1;
            }else return 1;
        }

    }

