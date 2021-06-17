package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.util.Log;

import org.apache.commons.lang.ObjectUtils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point3;

import java.util.ArrayList;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;
    private static Scalar borderColor;

    private static camera_params camparams;
    private static Mat camMatrix;
    private static Mat dstMatrix;
    private static float markerSize;

    private static Mat rvecs;
    private static Mat tvecs;
    private static Mat mean_rvecs;
    private static Mat rotationMatrix;

    private static Mat undistort_img;

    private static zbarQR zbarQR_obj;

    private static imgProcessing imgProc;

    public static void setCamCalibration() {
        camparams = new camera_params();
        camparams.process_camera();

        camMatrix=camparams.getCamMatrix();
        dstMatrix=camparams.getDistortionMat();
    }
    private Mat croppedImage(){
        Mat qr_img = api.getMatNavCam();

        imgProc = new imgProcessing();
        imgProc.findRectContours(qr_img);

        Log.d("QR","Contours complete");
        return imgProc.sharpenImg;
    }

    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree){

        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        Quaternion quaternion = new Quaternion((float)qx, (float)qy, (float)qz, (float)qw);

        return quaternion;
    }

    @Override
    protected void runPlan1(){
        api.startMission();
        setCamCalibration();
        boolean is_collusion =  moveToKOZ(11.0f,-8.6f,5.0f,0,0,-0.707,0.707);
        if(is_collusion){
            Log.d("move","to QR");
            Point3 QR_target = new Point3(11.21f, -9.8f, 4.79);
            moveToWrapper( QR_target.x,QR_target.y,QR_target.z,0,0,-0.707,0.707);
            Log.d("move","to QR complete");
        }
        else{
            moveToWrapper( 9.815f, -9.806f, 4.293f,1,0,0,0);

            Log.d("move","can't go");
        }
    }


    @Override
    protected void runPlan2(){
        api.startMission();
        setCamCalibration();

        // QR
        Point3 QR_target = new Point3(11.21f, -9.8f, 4.79);
        moveToWrapper( QR_target.x,QR_target.y,QR_target.z,0,0,-0.707,0.707);
        Log.d("QR","Start to read QR");

        // Mat imageCamera = api.getMatNavCam();
        Mat imageCamera = croppedImage();
        String qr_str = decodeQR(imageCamera);
        StringDecode QRData = new StringDecode();
        QRData.setString(qr_str);
        Log.d("QR","End to read QR");
        Log.d("QR", String.format("QR A : %d %.2f %.2f %.2f",QRData.getPattern(),QRData.getPosX(),QRData.getPosY(),QRData.getPosZ()));

        moveToWrapper( QRData.getPosX(),QRData.getPosY(),QRData.getPosZ(),0,0,-0.707,0.707); // go to A'
        Log.d("AR","Moved to A'");
        Mat ar_pic = api.getMatNavCam();
        Kinematics kinec_takepic = api.getTrustedRobotKinematics();
        Point pos_takepic = kinec_takepic.getPosition();

        //ARUCO
        ARmodel ArucoModel = new ARmodel();
        ArucoModel.estimate(ar_pic,camMatrix,dstMatrix);
        Log.d("AR", String.format("AR relative : %.2f %.2f %.2f",ArucoModel.getPosX(),ArucoModel.getPosY(),ArucoModel.getPosZ()));

        Point3 target_point = new Point3(pos_takepic.getX() + ArucoModel.getPosX(),pos_takepic.getY() + ArucoModel.getPosY(),pos_takepic.getZ() + ArucoModel.getPosZ());
        Log.d("AR",String.format("target point : %.3f %.3f %.3f",target_point.x,target_point.y,target_point.z));



        Point3 laser_offset = new Point3(-0.0572,0,0.1111);
        moveToWrapper(target_point.x + laser_offset.x,pos_takepic.getY(),target_point.z + laser_offset.z,0,0,-0.707,0.707);
        Log.d("AR","Moved to target point");


        api.laserControl(true);
        Log.d("QR", "laser");
        api.takeSnapshot();
        Log.d("QR", "take photo");

        moveToWrapper(10.505,-9,4.50, 0,0, -0.707, 0.707);
        moveToWrapper(10.505,-8.5,4.50, 0,0, -0.707, 0.707); // avoid KOZ2

        moveToWrapper(10.6, -8.0, 4.5,0, 0, -0.707, 0.707); //move to Point B



        api.reportMissionCompletion();
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    public String decodeQR(Mat qr_img)
    {

        zbarQR_obj = new zbarQR();
        zbarQR_obj.scanQRImage(imgProc.sharpenImg);
        Log.d("QR","readed : " + zbarQR_obj.qrCodeString);
        api.sendDiscoveredQR(zbarQR_obj.qrCodeString);

        return zbarQR_obj.qrCodeString;
    }


    public void moveToWrapper(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        final int LOOP_MAX = 2;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);
        int loopCounter = 0;
        while(!result.hasSucceeded()|| loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }
    }

    public boolean moveToKOZ(double pos_x, double pos_y, double pos_z,
                              double qua_x, double qua_y, double qua_z,
                              double qua_w){
        boolean collusion = false;
        final int LOOP_MAX = 2;
        final Point point = new Point(pos_x, pos_y, pos_z);

        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, false);


//        Log.d("move", String.format("result : %s %d",result.getMessage(),result.getStatus()));

        System.out.print("get message ");
        System.out.println(result.getMessage());
        System.out.print("get status");
        System.out.println(result.getStatus());

        int loopCounter = 0;
        while(!result.hasSucceeded()){
            result = api.moveTo(point, quaternion, false);
            ++loopCounter;
            if (result.getStatus() ==  Result.Status.EXEC_FAILED) {
                Log.d("move", String.format("collusion = true (NULL) loop_cnt: %d", loopCounter));
                return true;
            }
        }
//        Log.d("move", String.format("collusion = true (NULL) loop_cnt: %d", loopCounter));

        return false;

//        int loopCounter = 0;
//        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
//                result = api.moveTo(point, quaternion, false);
//                ++loopCounter;
//        }



    }

}

