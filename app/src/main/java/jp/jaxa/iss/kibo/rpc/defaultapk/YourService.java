package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.util.Log;

import com.yanzhenjie.zbar.Config;
import com.yanzhenjie.zbar.Image;
import com.yanzhenjie.zbar.ImageScanner;
import com.yanzhenjie.zbar.Symbol;
import com.yanzhenjie.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point3;

import java.util.ArrayList;

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

        Quaternion Q1 = eulerToQuaternion(0,0,0);
        moveToWrapper( 9.9,-9.806,4.293,Q1.getX(),Q1.getY(),Q1.getZ(),Q1.getW());

        Quaternion Q2 = eulerToQuaternion(90,0,0);
        moveToWrapper( 9.9,-9.806,4.293,Q2.getX(),Q2.getY(),Q2.getZ(),Q2.getW());

        Quaternion Q5 = eulerToQuaternion(0,0,0);
        moveToWrapper( 9.9,-9.806,4.293,Q5.getX(),Q5.getY(),Q5.getZ(),Q5.getW());

        Quaternion Q3 = eulerToQuaternion(0,90,0);
        moveToWrapper( 9.9,-9.806,4.293,Q3.getX(),Q3.getY(),Q3.getZ(),Q3.getW());

        Quaternion Q6 = eulerToQuaternion(0,0,0);
        moveToWrapper( 9.9,-9.806,4.293,Q6.getX(),Q6.getY(),Q6.getZ(),Q6.getW());

        Quaternion Q4 = eulerToQuaternion(0,0,90);
        moveToWrapper( 9.9,-9.806,4.293,Q4.getX(),Q4.getY(),Q4.getZ(),Q4.getW());

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


        //ARUCO
        ARmodel ArucoModel = new ARmodel();
        ArucoModel.estimate(imgProc.processedImg,camMatrix,dstMatrix);
        Point3 AR_target = new Point3( QR_target.x + ArucoModel.getPosX(),QR_target.y + ArucoModel.getPosY() ,QR_target.z + ArucoModel.getPosZ());
        Log.d("QR", String.format("AR relative : %.2f %.2f %.2f",ArucoModel.getPosX(),ArucoModel.getPosY(),ArucoModel.getPosZ()));
        Log.d("QR", String.format("AR absolute : %.2f %.2f %.2f",AR_target.x,AR_target.y,AR_target.z));

        Point3 goal_target =  new Point3( QR_target.x + ArucoModel.getPosX(),QR_target.y  ,QR_target.z + ArucoModel.getPosZ());
        moveToWrapper( QRData.getPosX(),QRData.getPosY(),QRData.getPosZ(),0,0,-0.707,0.707);
        Log.d("QR", "move to A-");

        api.takeSnapshot();
        Log.d("QR", "take photo");
        api.laserControl(true);
        Log.d("QR", "laser");



        api.reportMissionCompletion();
    }


    @Override
    protected void runPlan2(){
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

}

