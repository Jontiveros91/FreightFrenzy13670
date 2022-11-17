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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a mecanum chassis.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "left_front"
 * Motor channel:  Right front drive motor:        "right_front"
 * Motor channel:  Left back drive motor:        "left_back"
 * Motor channel:  Right back drive motor:        "right_back"
 */
public class HardwareMecanum
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack    = null;
    public DcMotor  rightBack    = null;
    public DcMotor lift = null;

    public CRServo spinner = null;
    public CRServo roller = null;
    public Servo   bay    = null;

    static final double     COUNTS_PER_MOTOR_REV    = 312 ;    // eg: GoBilda 312 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 40;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    Telemetry telemetry;
    private ElapsedTime movetime  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telemetry;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftBack  = hwMap.get(DcMotor.class, "left_back");
        rightBack = hwMap.get(DcMotor.class, "right_back");
        lift = hwMap.get(DcMotor.class, "lift");

        spinner = hwMap.get(CRServo.class, "spinner");
        roller = hwMap.get(CRServo.class, "roller");
        bay    = hwMap.get(Servo.class, "bay");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rest();

        }

    public void rest() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void noEncoder(){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runEncoder(){
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void toPosition(){
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goForward(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

    public void goRight(double speed){
        leftFront.setPower(-speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
    }

    public void turnRight(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }

    public boolean allBusy(){
        return (leftFront.isBusy()&&leftBack.isBusy()&&rightFront.isBusy()&&rightBack.isBusy());
    }

    public boolean anyBusy(){
        return (leftFront.isBusy()||leftBack.isBusy()||rightFront.isBusy()||rightBack.isBusy());
    }


    public void forwardInches(double speed, double inches, double timeoutS) {
        //clear current position and start run encoder
        reset();
        // Determine new target position, and pass to motor controller
        int newTarget = leftFront.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        movetime.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((movetime.seconds() < timeoutS) && allBusy()) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rest();

        // reset encoder
        reset();

    }

    public void rightInches(double speed, double inches, double timeoutS) {
        //clear current position and start run encoder
        reset();
        // Determine new target position, and pass to motor controller
        int newTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * 1.4);

        leftFront.setTargetPosition(-1 * newTarget);
        rightFront.setTargetPosition(-1 * newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        movetime.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((movetime.seconds() < timeoutS) && allBusy()) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rest();

        // reset encoder
        reset();

    }

    public void turnDegrees(double speed, int degrees, double timeoutS) {
        //clear current position and start run encoder
        reset();
        // Determine new target position, and pass to motor controller
        int newTarget = leftFront.getCurrentPosition() + (int) (degrees/4.1 * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(-1 * newTarget);
        leftBack.setTargetPosition(newTarget);
        rightBack.setTargetPosition(-1 * newTarget);

        // Turn On RUN_TO_POSITION
        toPosition();

        // reset the timeout time and start motion.
        movetime.reset();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((movetime.seconds() < timeoutS) && allBusy()) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d", leftFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rest();

            // reset encoder
            reset();

    }

    public void reset(){
        resetEncoder();
        runEncoder();
    }
}

