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

/**
 * game pad 1:
 * move/turn - left stick
 * strafe - right stick
 * foundation up - button 1
 * foundation down - button 2
 *
 * game pad 2:
 * arm up/dn and in/out w/o changing end effector orientation - left stick
 * end effector manual level and open/close - right stick (probably not used)
 * Presets:
 * arm and end effector move to pick-up position - button 1
 * close end effector,  pick-up stone, collapse to fit under bridge - button 2 (or reuse button 1)
 * <<after picking up stone, should we roll the end effetor back so that gravity is not working against us?>>
 * end effector open - button 3
 * end effector close - button 4
 * retract arm to fit under bridge (only needed if button 1 does not fit under bridge) - button 5
 */


@TeleOp(name = "Tele1", group = "TeleOp")
//@Disabled
public class Tele1 extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot = new THardware1();   // Use a hardware
    ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;
    double target_x, current_x, now_seconds;

    ArtArm reach = new ArtArm(14.5, 15.75, 3, 1, 1, 2);

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

        double x = 2;
        double y = 1;
        double[] pos;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Initialize gyro stuff
        BNO055IMU.Parameters parameters = initIMUParams();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //The "flat" robot angle
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        target_x = angles.firstAngle;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /******************************************* BASE MOVE **********************************************************/

            SFB = -gamepad1.left_stick_y; /*The joystick goes negative when pushed forwards, so negate it*/
            SRL = gamepad1.right_stick_x; // Check sign
            T = -gamepad1.left_stick_x;

            // Run wheels in POV mode

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

            /******************************************* ARM MOVE **********************************************************/

            if(x > reach.maxX())
                x = reach.maxX();
            if(x < 0)
                x = 0;
            else
                x += gamepad2.left_stick_x;


            if(y > reach.maxY())
                y = reach.maxY();
            if(y < 0)
                y = 0;
            else
                y -= gamepad1.left_stick_y;

            if(gamepad2.y){
                x = 2;
                y = 1;
            }

            pos = reach.goToXY(x, y);
            robot.ArmBase.setTargetPosition((int) pos[0]);
            robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmBase.setPower(0.3);
            robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
            robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmJoint.setPower(0.3);
            if(Math.abs(gamepad2.right_stick_y) < 0.1)
                robot.EndJoint.setPosition((-gamepad2.right_stick_y + 2) / 2);
            else
                robot.EndJoint.setPosition(pos[2]/(1.5*Math.PI));

            /********************************************* GRIPPER **********************************************************/

            if(gamepad2.b){
                robot.Gripper.setPosition(1.0);
            } else if(gamepad2.a){
                robot.Gripper.setPosition(0.0);
            }

            /******************************************* FOUNDATION **********************************************************/

            if (gamepad1.left_bumper)
                robot.FoundationMover.setPosition(.2);
            else if(gamepad1.right_bumper)
                robot.FoundationMover.setPosition(.9);


            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
        }
    }


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




}