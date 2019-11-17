package org.firstinspires.ftc.teamcode;

/**
 * Created by Miranda on 10/20/2017.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "PID Test", group = "TeleOp")
//@Disabled
public class PID_Test extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot = new THardware1();   // Use a hardware
    ElapsedTime runtime = new ElapsedTime();
    /*******  NEW *********/
    //Declarations for the tipping-control part
    BNO055IMU imu;
    Orientation angles;
    double target_x, current_x, now_seconds;
    /*******  END *********/

    //Double for more precision
    @Override
    public void runOpMode() throws InterruptedException {

        double MFR;
        double MFL;
        double MBR;
        double MBL;

        double SFB;
        double SRL;
        double T;

        double max;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double dir = 0, speed = 0.5, dist = 10000;
        double Kp = 0.1, Ki = 0, dt = 0.01;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up)
                dir = 0;
            else if(gamepad1.dpad_right)
                dir = 90;
            else if(gamepad1.dpad_down)
                dir = 180;
            else if(gamepad1.dpad_left)
                dir = 270;

            if(Kp > 0.01 && gamepad1.left_bumper)
                Kp -= 0.01;
            else if(Kp <= 0.99 && gamepad1.right_bumper)
                Kp += 0.01;

            if(Ki >= 0.01 && gamepad1.left_trigger > 0.5)
                Ki -= 0.01;
            else if(Ki <= 0.99 && gamepad1.right_trigger > 0.5)
                Ki += 0.01;

            if(speed >= 0 && gamepad1.left_stick_y > .5)
                speed -= 0.01;
            else if(speed <= 0.99 && gamepad1.left_stick_y < -.5)
                speed += 0.01;

            if(dist >= 0 && gamepad1.right_stick_y > .5)
                dist -= 100;
            else if(dist <= 10000 && gamepad1.right_stick_y < -.5)
                dist += 100;

            telemetry.addData("Distance", dist);
            telemetry.addData("Direction", dir);
            telemetry.addData("Speed", speed);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.update();

            if(gamepad1.a) {
                telemetry.addData("Start!", "");
                telemetry.update();
                sleep(2000);
                PIDMove(dir, speed, dist, Kp, Ki, 0.01);
                sleep(1000);
            }

            // Pause for metronome tick.  100 mS each cycle = update 10 times a second.
            robot.waitForTick(100);
        }
    }


    /*******  NEW *********/

    //Tipping-check variables and methods

    private final double ANGLE_THRESHOLD = 20;  //Angle to decide tipping has occurred
    private final double TIME_THRESHOLD = 10;   //Minimum seconds between tipping reports
    private boolean tipping = false;  //Current state of robot
    private double last_tip_time = 0;
    private ElapsedTime tip_timer = new ElapsedTime();

    /*
    This method returns true only once when the robot goes from non-tipped to tipped. If the
    robot remains tipped, it returns false. It returns true no more than once every TIME_THRESHOLD
    seconds.
     */
    private boolean checkTip(double pitch) {
        boolean result;
        boolean exceed = Math.abs(pitch) > ANGLE_THRESHOLD;
        //Check if the robot is going from "not tipping" to "tipping" after TIME_THRESHOLD
        if ( tipping == false && exceed == true && tip_timer.seconds() > last_tip_time+TIME_THRESHOLD){
            result = true;  //Report that tipping has occurred
            last_tip_time = tip_timer.seconds();
        }
        else{
            result = false;
        }
        tipping = exceed;
        return result;
    }

    private BNO055IMU.Parameters initIMUParams(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        return parameters;
    }
    /*******  END *********/

    public void PIDMove(double dir, double speed, double dist, double Kp, double Ki, double dt){
        /*
        dir = approx direction in degrees to move relative to forward face of robot.
        speed = speed to move [0,1]
        dist = approx distance to move in cm.
        Kp = proportional gain > 0
        Ki = integral gain >= 0
        dt = is a time step to control how fast the loop runs.
        */
        dir = dir*Math.PI / 180.0;
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        int FRinit = robot.MFR.getCurrentPosition();
        int FLinit = robot.MFL.getCurrentPosition();
        int BRinit = robot.MBR.getCurrentPosition();
        int BLinit = robot.MBL.getCurrentPosition();
        double drive = speed*Math.cos(dir);
        double strafe = speed*Math.sin(dir);

        double yaw_init = angles.thirdAngle; //get current yaw angle when the move is requested.  Goal is to keep this angle constant.
        double estX, estY, estDist;
        double yaw, yawErr, yawErrI;
        double turn = 0;
        int count = 0;
        do{
            count ++;
            //get motor encoder values and compute estimated distance, estDist
            estX = ( (robot.MFR.getCurrentPosition()-FRinit) + (robot.MFL.getCurrentPosition()-FLinit) +
                    (robot.MBR.getCurrentPosition()-BRinit) + (robot.MBL.getCurrentPosition()-BLinit)) * Math.cos(45*Math.PI/180.0);
            estY = (-(robot.MFR.getCurrentPosition()-FRinit) + (robot.MFL.getCurrentPosition()-FLinit) +
                    (robot.MBR.getCurrentPosition()-BRinit) - (robot.MBL.getCurrentPosition()-BLinit)) * Math.sin(45*Math.PI/180.0);
            estDist = Math.sqrt(Math.pow(estX, 2) + Math.pow(estY, 2));

            yawErrI= 0;
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw = angles.thirdAngle;
            yawErr = yaw - yaw_init;  //note, some math may be needed so that yawErr does not make 360 deg jumps.
            // Alternatively, set gyro yaw to zero instead of measuring ywa_init

            if(Math.abs(yawErr) < 45) { //prevent 180 wraparound, but also limit how how big this term can get.
                yawErrI = yawErrI + yawErr * dt;  //this is effectively the integral of yaw error
                turn = Kp * (yawErr) + Ki * (yawErrI);  //correction you make to compensate for the twist of the foundation
            }
            if (Math.abs(turn) > 1.0) {
                turn = turn / Math.abs(turn);
            }
            moveWithEnc(drive, strafe, turn, speed);
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("Speed", speed);
            telemetry.addData("Yaw", yaw);
            telemetry.addData(dist + " ", estDist);
            telemetry.addData("", count);
            telemetry.update();
            sleep((int) (dt*1000));
        } while(estDist < dist);

    }

    private void moveWithEnc(double drive, double strafe, double turn, double speed){
        double FR =  drive - turn -  strafe; //+-
        double BR =  drive - turn +  strafe; //++
        double FL =  drive + turn +  strafe; //-+
        double BL =  drive + turn -  strafe; //--

        double Mmax = Math.max(Math.max(Math.abs(FR), Math.abs(BR)), Math.max(Math.abs(FL), Math.abs(BL)));

        FR =  speed*FR / Mmax;
        BR =  speed*BR / Mmax;
        FL =  speed*FL / Mmax;
        BL =  speed*BL / Mmax;

        robot.MFL.setPower(FL);
        robot.MBL.setPower(BL);
        robot.MFR.setPower(FR);
        robot.MBR.setPower(BR);
    }

}