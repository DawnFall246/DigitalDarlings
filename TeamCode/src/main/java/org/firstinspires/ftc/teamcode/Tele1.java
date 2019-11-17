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

@TeleOp(name = "Tele1", group = "TeleOp")
//@Disabled
public class Tele1 extends LinearOpMode {

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

        /*******  NEW *********/
        //Initialize gyro stuff
        BNO055IMU.Parameters parameters = initIMUParams();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        /*******  END *********/

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*******  NEW *********/
        //The "flat" robot angle
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        target_x = angles.firstAngle;
        /*******  END *********/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ////////////////////////////////////////////BASE MOVE/////////////////////////////////////////////////////////

            SFB = -gamepad1.right_stick_y; /*The joystick goes negative when pushed forwards, so negate it*/
            SRL = gamepad1.right_trigger - gamepad1.left_trigger;
            T = -gamepad1.right_stick_x;
            // Run wheels in POV mode
            // In this mode the Left stick moves the robot fwd and back and turns left and right.

            MFR = (SFB + T - SRL) / 1.5; //controls base motion
            MFL = (SFB - T + SRL) / 1.5; //slows down robot to make it more manageable
            MBR = (SFB + T + SRL) / 1.5;
            MBL = (SFB - T - SRL) / 1.5;

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
                while ( runtime.seconds() < now_seconds + 0.5);
            }
            /*******  END *********/


            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
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


}