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

    @Override
    protected void runPlan1(){
        api.startMission();
        setCamCalibration();

        // QR
        moveToWrapper(11.21,-9.8,4.79,0,0,-0.707,0.707);
        Log.d("QR","Start to read QR");
        // Mat imageCamera = api.getMatNavCam();
        Mat imageCamera = croppedImage();
        String qr_str = decodeQR(imageCamera);
        StringDecode QRData = new StringDecode();
        QRData.setString(qr_str);
        Log.d("QR","End to read QR");
        Log.d("QR","x : " + String.valueOf(QRData.x));

        //ARUCO
        //ARmodel ArucoModel = new ARmodel();
        //ArucoModel.estimate(imgProc.processedImg,camMatrix,dstMatrix);




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

