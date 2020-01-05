/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


@Autonomous(name = "Auto1Blue", group = "Auto")
//@Disabled
public class AutoRedFoundation extends LinearOpMode {

    AHardware1 robot = new AHardware1();
    private ElapsedTime runtime = new ElapsedTime();

    Orientation angles;
    Acceleration gravity;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    private static final String VUFORIA_KEY = "";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        //initVuforia();

        robot.init(hardwareMap);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        /*
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        */
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        runtime.reset();
        waitForStart();
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        sleep(1000);
        telemetry.addData("Start", "");
        telemetry.update();
        sleep(500);
        //gyroMoveStraight("r", 2000);

        //Move blue foundation

        // Move straight 87 cm
        PIDMove(0, 0.8, 87, .25, 0.15, .01);
        // turn 90 degrees clockwise
        turn(-90);
        //move straight 143.515 cm
        PIDMove(0, 0.8, 143.515, .25, 0.15, .01);
        // turn 90 degrees clockwise
        turn(-90);
        // bring up flats
        robot.FoundationMover.setPosition(0.17);
        //move forward 0.21 cm on foundation mover
        PIDMove(0, 0.8, 0.21, .25, 0.15, .01);
        // bring down flats
        robot.FoundationMover.setPosition(0.67);
        //strafe 143.515 cm
        PIDMove(-90, 0.8, 143.515, 0.1, 0, 0.01);
        // bring up flats and let go of foundation mover
        robot.FoundationMover.setPosition(0.17);
        // return under bridge
        PIDMove(0, 0.8, 87.21, .25, 0.15, .01);

        telemetry.addData("Finish", "");
        robot.MBR.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MFL.setPower(0);
        telemetry.update();
        sleep(1000);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;////////////////////////////////////////////////////////////////////////////////////////////////////
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }


    public void toColor() {
        ///////////////////////////////////////////////////////////////COLOR///////////////////////////////////////////////////////////////

        int red, green, blue, alpha;

        double dR, dB, dM;

        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setPower(.5);
        robot.MBL.setPower(.5);
        robot.MFR.setPower(.5);
        robot.MFL.setPower(.5);

        runtime.reset();
        do {
            //compute 'distance' from current color sensor reading to RED, BLUE and MAT

            red = robot.Color.red();
            green = robot.Color.green();
            blue = robot.Color.blue();
            alpha = robot.Color.alpha();

            dR = Math.sqrt(Math.pow((alpha - 122), 2) + Math.pow((red - 90), 2) + Math.pow((green - 23), 2) + Math.pow((blue - 26), 2));
            dB = Math.sqrt(Math.pow((alpha - 106), 2) + Math.pow((red - 18), 2) + Math.pow((green - 38), 2) + Math.pow((blue - 58), 2));
            dM = Math.sqrt(Math.pow((alpha - 103), 2) + Math.pow((red - 39), 2) + Math.pow((green - 37), 2) + Math.pow((blue - 31), 2));
            telemetry.addData("Alpha: ", alpha);
            telemetry.addData("Red: ", red);
            telemetry.addData("Green: ", green);
            telemetry.addData("Blue: ", blue);
            telemetry.addData("dB < dM = ", dB < dM);
            telemetry.addData("dR < dM = ", dR < dM);
            telemetry.update();
        } while (opModeIsActive() && dB > dM && dR > dM && runtime.seconds() < 3);

        robot.MBR.setPower(0.0);
        robot.MBL.setPower(0.0);
        robot.MFR.setPower(0.0);
        robot.MFL.setPower(0.0);

        sleep(100);
    }

    public void turn(double init_target_angle) {
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double yaw_theta = angles.thirdAngle;

        double target_angle = init_target_angle + yaw_theta;
        if(target_angle > 180)
            target_angle -= 360;
        else if(target_angle < -180)
            target_angle += 360;

        double left_power, right_power;

        while (Math.abs(yaw_theta - target_angle) > 1  &&  opModeIsActive()) {
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw_theta = angles.thirdAngle;
            if (init_target_angle > 0) { //Turn left
                left_power = -0.4;
                right_power = 0.4;
            } else { //Turn right
                left_power = 0.4;
                right_power = -0.4;
            }

            robot.MFL.setPower(left_power);
            robot.MBL.setPower(left_power);
            robot.MFR.setPower(right_power);
            robot.MBR.setPower(right_power);

            telemetry.addData("Angle: ", angles.thirdAngle);
            telemetry.addData("Diff: ", Math.abs(yaw_theta - target_angle));
            telemetry.update();
        }

        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);

        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnTo(double init_target_angle) {
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double yaw_theta = angles.thirdAngle;
        double target_angle = init_target_angle;
        if(init_target_angle > 180)
            target_angle -= 360;
        else if(init_target_angle < -180)
            target_angle += 360;

        double left_power, right_power;

        while (Math.abs(yaw_theta - init_target_angle) > 1  &&  opModeIsActive()) {
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw_theta = angles.thirdAngle;
            if (target_angle - yaw_theta < 0) { //Turn right(-)
                left_power  = 0.5;
                right_power = -0.5;
            } else { //Turn left(+)
                left_power  = -0.5;
                right_power = 0.5;
            }

            robot.MFL.setPower(left_power);
            robot.MBL.setPower(left_power);
            robot.MFR.setPower(right_power);
            robot.MBR.setPower(right_power);

            telemetry.addData("Angle: ", angles.thirdAngle);
            telemetry.addData("Target", target_angle);
            telemetry.addData("Init", init_target_angle);
            telemetry.addData("Diff: ", yaw_theta - target_angle);
            telemetry.update();
        }

        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);

        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void gyroMoveStraight(String dir, int encoder) {
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("dir", dir);
        telemetry.update();
        if (dir.equals("f") || dir.equals("b")) {
            while (Math.abs(Math.abs(robot.MFL.getCurrentPosition()) - encoder) > 5) {
                angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double left_power, right_power;
                double yaw_theta = angles.thirdAngle;
                telemetry.addData("Yaw value", yaw_theta);
                if (yaw_theta < -2) {           //Turning right
                    left_power = 0.4;
                    right_power = 0.8;
                } else if (yaw_theta > 2) {     //Turning left
                    left_power = 0.8;
                    right_power = 0.4;
                } else {                        //Going straight
                    left_power = 0.6;
                    right_power = 0.6;
                }

                if (dir.equals("f")) {
                    robot.MFL.setPower(left_power);
                    robot.MBL.setPower(left_power);
                    robot.MFR.setPower(right_power);
                    robot.MBR.setPower(right_power);
                } else if (dir.equals("b")) {
                    robot.MFL.setPower(-right_power);
                    robot.MBL.setPower(-right_power);
                    robot.MFR.setPower(-left_power);
                    robot.MBR.setPower(-left_power);
                }
            }
        } else if (dir.equals("l") || dir.equals("r")) {
            while (Math.abs(Math.abs(robot.MFL.getCurrentPosition()) - encoder) > 5) {
                angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double front_power, back_power;
                double yaw_theta = angles.thirdAngle;
                telemetry.addData("Yaw value", yaw_theta);
                if (yaw_theta < -2) {           //Turning right
                    back_power = 0.4;
                    front_power = 0.8;
                } else if (yaw_theta > 2) {     //Turning left
                    back_power = 0.8;
                    front_power = 0.4;
                } else {                        //Going straight
                    back_power = 0.6;
                    front_power = 0.6;
                }

                if (dir.equals("r")) {
                    robot.MFL.setPower(front_power);
                    robot.MBL.setPower(-back_power);
                    robot.MFR.setPower(-front_power);
                    robot.MBR.setPower(back_power);
                } else if (dir.equals("l")) {
                    robot.MFL.setPower(-back_power);
                    robot.MBL.setPower(front_power);
                    robot.MFR.setPower(back_power);
                    robot.MBR.setPower(-front_power);
                }

                telemetry.addData("Front", robot.MFL.getPower());
                telemetry.addData("Back", robot.MBL.getPower());
                telemetry.addData("To end", Math.abs(robot.MFL.getCurrentPosition() - encoder));
                telemetry.update();
            }
            telemetry.addData("done", "");
            telemetry.update();

            robot.MFL.setPower(0);
            robot.MBL.setPower(0);
            robot.MFR.setPower(0);
            robot.MBR.setPower(0);
        }
    }

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

    private void telemetryUpdate(int number){
        telemetry.addData("", number);
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.update();
    }

    public void waitUntilPosition(DcMotor motor, int sec){
        runtime.reset();
        while  (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > 15 && opModeIsActive()){
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.update();
            if(runtime.seconds() >= sec)
                break;
        }
    }
    public void waitUntilWheelPosition(){
        while((Math.abs(robot.MBR.getCurrentPosition() - robot.MBR.getTargetPosition()) > 15 ||
                Math.abs(robot.MFL.getCurrentPosition() - robot.MFL.getTargetPosition()) > 15 ||
                Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) > 15 ||
                Math.abs(robot.MFR.getCurrentPosition() - robot.MFR.getTargetPosition()) > 15) &&
                opModeIsActive())
        {
            if(Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) <= 15){
                robot.MBL.setPower(0);
            }
        }
    }
    public void resetWheelEncoders(){
        robot.MFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(100);
    }

    public void move(double inches, double power, int time){
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MBR.setTargetPosition(robot.MBR.getCurrentPosition() + (int) ((5000/55.0)*inches));
        robot.MBL.setTargetPosition(robot.MBL.getCurrentPosition() + (int) ((5000/55.0)*inches));
        robot.MFR.setTargetPosition(robot.MFR.getCurrentPosition() + (int) ((5000/55.0)*inches));
        robot.MFL.setTargetPosition(robot.MFL.getCurrentPosition() + (int) ((5000/55.0)*inches));
        robot.MFL.setPower(power);
        robot.MFR.setPower(power);
        robot.MBL.setPower(power);
        robot.MBR.setPower(power);
        waitUntilPosition(robot.MBL, time);
        waitUntilPosition(robot.MBR, time);
        if(Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) <= 15){
            robot.MBL.setPower(0);
        }
        waitUntilPosition(robot.MFR, time);
        if(Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) <= 15){
            robot.MBL.setPower(0);
        }
        waitUntilPosition(robot.MFL, time);
        if(Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) <= 15){
            robot.MBL.setPower(0);
        }
    }
    public void strafeLeft(double inches, double power, int time){
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MBR.setTargetPosition(robot.MBR.getCurrentPosition() - (int) (5000/49.75*inches));
        robot.MBL.setTargetPosition(robot.MBL.getCurrentPosition() + (int) (5000/49.75*inches));
        robot.MFR.setTargetPosition(robot.MFR.getCurrentPosition() + (int) (5000/49.75*inches));
        robot.MFL.setTargetPosition(robot.MFL.getCurrentPosition() - (int) (5000/49.75*inches));
        robot.MBR.setPower(power);
        robot.MBL.setPower(power);
        robot.MFR.setPower(power);
        robot.MFL.setPower(power);
        waitUntilPosition(robot.MBL, time);
        waitUntilPosition(robot.MBR, time);
        waitUntilPosition(robot.MFL, time);
        waitUntilPosition(robot.MFR, time);
    }
    public void slowStop(DcMotor motor){
        double power = motor.getPower();
        while(Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.update();
            if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < 250 && opModeIsActive()) {
                motor.setPower(power / 4);
            }else if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < 500 && opModeIsActive()) {
                motor.setPower(power / 2);
            }
        }
    }
}