package org.firstinspires.ftc.teamcode;

/**
 * Created by Miranda on 10/20/2017.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "NoTip2", group = "TeleOp")
//@Disabled
public class NoTip2 extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot = new THardware1();   // Use a hardware
    ArtArm reach = new ArtArm(15, 16.5, 3, 1, 1, 2);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();
    /*******  NEW *********/
    //Declarations for the tipping-control part
    BNO055IMU imu;
    Orientation angles;
    double target_x, current_x, now_seconds;

    //Tipping-check variables and methods

    final double ANGLE_THRESHOLD = 15;  //Angle to decide tipping has occurred
    final double TIME_THRESHOLD = 5;   //Minimum seconds between tipping reports
    boolean tipping = false;  //Current state of robot
    double last_tip_time = 0;
    ElapsedTime tip_timer = new ElapsedTime();
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

        int L = 0;
        int state;
        int lifted = 0;

        double max;

        double current = 0;
        double adjust;

        double[] pos99 = {0, 0, 0.53};

        double[] pos10 = {-220, -351, 0.531};
        double[] pos15 = {-320, -310, .40};
        double[] pos1 = {-340, -80, 0.47};

        double[] pos20 = {-468, -730, 0.57};
        double[] pos25 = {-571, -700, 0.40};
        double[] pos2 = {-591, -421, 0.51};

        double[] pos30 = {-1008, -1990, 0.67};
        double[] pos35 = {-1070, -1900, 0.57};
        double[] pos3 = {-1090, -1830, 0.67};

        double[] pos100 = {-163, -2740, 0.09};

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /*******  NEW *********/
        //Initialize gyro stuff
        BNO055IMU.Parameters parameters = initIMUParams();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        /*******  END *********/

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        state = 10;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*******  NEW *********/
        //The "flat" robot angle
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        target_x = angles.firstAngle;
        /*******  END *********/

        // run until the end of the match (driver presses STOP)
        if(opModeIsActive())
            totalTime.reset();
        while (opModeIsActive()) {
            ////////////////////////////////////////////BASE MOVE/////////////////////////////////////////////////////////

            T = -gamepad1.right_stick_x;
            SFB = -gamepad1.right_stick_y; /*The joystick goes negative when pushed forwards, so negate it*/
            SRL = gamepad1.right_trigger - gamepad1.left_trigger;

            //Noise control
            if(Math.abs(T) < 0.1)
                T = 0;
            if(Math.abs(SFB) < 0.1)
                SFB = 0;

            // Run wheels in POV mode
            // In this mode the Left stick moves the robot fwd and back and turns left and right.
            MFR = (SFB + T - SRL) / 1.25; //controls base motion
            MFL = (SFB - T + SRL) / 1.25; //slows down robot to make it more manageable
            MBR = (SFB + T + SRL) / 1.25;
            MBL = (SFB - T - SRL) / 1.25;


            telemetry.update();

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(MFR), Math.abs(MFL));
            if (max > 1.0) {
                MFR /= max; //can't exceed 1
                MFL /= max;
                MBR /= max;
                MBL /= max;
            }

            robot.MFR.setPower(MFR);
            robot.MFL.setPower(MFL);
            robot.MBR.setPower(MBR);
            robot.MBL.setPower(MBL);

            telemetry.addData("MBR", robot.MBR.getCurrentPosition()); //Printing motor position on phone
            telemetry.addData("MBL", robot.MBL.getCurrentPosition());
            telemetry.addData("MFR", robot.MFR.getCurrentPosition());
            telemetry.addData("MFL", robot.MFL.getCurrentPosition());

            ////////////////////////////////////////////ARTICULATED ARM/////////////////////////////////////////////////////////
            while (gamepad1.x) {
                telemetry.addData("X: ", gamepad1.x);
                telemetry.update();
                if (!gamepad1.x) {
                    if (state == 99)
                        state = 10;
                    else if (state == 10)
                        state = 20;
                    else if (state == 20)
                        state = 30;
                    else if (state == 30)
                        state = 10;

                    if (state == 15)
                        state = 25;
                    else if (state == 25)
                        state = 35;
                    else if (state == 35)
                        state = 15;

                    if (state == 100)
                        state = 10;
                    if (state == 200)
                        state = 20;
                    if (state == 300)
                        state = 30;
                }
            }
            while (gamepad1.b) {
                telemetry.addData("B: ", gamepad1.b);
                telemetry.update();
                if (!gamepad1.b) {
                    if (state == 10)
                        state = 15;
                    else if (state == 15)
                        state = 10;

                    if (state == 20)
                        state = 25;
                    else if (state == 25)
                        state = 20;

                    if (state == 30)
                        state = 35;
                    else if (state == 35)
                        state = 30;
                }
            }
            while (gamepad1.y) {
                telemetry.addData("Y: ", gamepad1.y);
                telemetry.update();
                if (!gamepad1.y) {
                    if (state == 10 || state == 15)
                        state = 100;
                    if (state == 20 || state == 25)
                        state = 200;
                    if (state == 30 || state == 35)
                        state = 300;
                }
            }
            /*
            if(state % 10 == 5){
                if(gamepad1.a && state % 10 == 5)
                    robot.Rollers.setPower(1.0);
                else
                    robot.Rollers.setPower(0.04);
            }else {*/
                while (gamepad1.a) {
                    telemetry.addData("A: ", gamepad1.a);
                    telemetry.update();
                    if (state >= 100)
                    if (!gamepad1.a) {
                        if (state == 10) {
                            armMove(pos1);
                            armMove(pos10);
                        }
                        if (state == 20) {
                            armMove(pos2);
                            armMove(pos20);
                        }
                        if (state == 30) {
                            armMove(pos3);
                            armMove(pos30);
                        }
                    }
                }
            //}

            if (state == 99)
                armMove(pos99);
            if (state == 10)
                armMove(pos10);
            if (state == 20)
                armMove(pos20);
            if (state == 30)
                armMove(pos30);
            if (state == 15)
                armMove(pos15);
            if (state == 25)
                armMove(pos25);
            if (state == 35)
                armMove(pos35);
            if (state >= 100)
                armMove(pos100);



            /*******  NEW *********/
            //The part that checks to see if the robot is tipping
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            current_x = angles.firstAngle;
            if ( checkTip(target_x - current_x)){ //If the robot is tipping
                //turn off the motors
                robot.MFR.setPower(0);
                robot.MFL.setPower(0);
                robot.MBR.setPower(0);
                robot.MBL.setPower(0);

                //Wait for 0.5 second
                now_seconds = runtime.seconds();
                while ( runtime.seconds() < now_seconds + 1){
                    telemetry.addData("Tipping...", "");
                    telemetry.update();
                }
            }
            /*******  END *********/

            telemetry.addData("Time Left: ", 2*60 - totalTime.seconds());
            telemetry.addData("Base: ", robot.ArmBase.getCurrentPosition());
            telemetry.addData("Joint: ", robot.ArmJoint.getCurrentPosition());
            telemetry.addData("Servo:", robot.EndJoint.getPosition());
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
        }
    }

    public void armMove(double[] pos) {
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setPower(0.1);
        robot.ArmJoint.setTargetPosition((int) pos[1]);
        robot.ArmJoint.setPower(0.2);
        robot.EndJoint.setPosition(pos[2]);
    }



    /*****NEW******/
    /*
    This method returns true only once when the robot goes from non-tipped to tipped. If the
    robot remains tipped, it returns false. It returns true no more than once every TIME_THRESHOLD
    seconds.
     */
    private boolean checkTip(double pitch) {
        boolean result;
        boolean exceed = Math.abs(pitch) > ANGLE_THRESHOLD;
        //Check if the robot is going from "not tipping" to "tipping" after TIME_THRESHOLD
        if ( !tipping && exceed && tip_timer.seconds() > last_tip_time+TIME_THRESHOLD){
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
    //Foundation Mover
    public void moveFoundation(){

    }
    /*******  END *********/
}