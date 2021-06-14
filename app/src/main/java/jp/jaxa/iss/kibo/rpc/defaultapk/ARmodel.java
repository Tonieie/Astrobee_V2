package jp.jaxa.iss.kibo.rpc.defaultapk;



import java.util.ArrayList;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ARmodel {

    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;
    private static Scalar borderColor;


    private static float markerSize;

    private static Mat rvecs;
    private static Mat tvecs;
    private static Mat mean_rvecs;
    private static Mat rotationMatrix;


    public void estimateTest(Mat img, Mat camMatrix, Mat dstMatrix) {
        long total_time = 0;
        int loop_cnt = 50;
        for (int i = 0; i < loop_cnt; i++) {
            long start_time = System.currentTimeMillis();
            estimate(img, camMatrix, dstMatrix);
            long end_time = System.currentTimeMillis();
            long diff_time = end_time - start_time;
            total_time += diff_time;
            System.out.printf("%d: AR time : %d ms\n", i, (diff_time) * 200);
        }


        System.out.printf("AR time : %d ms\n", (total_time) * 200 / loop_cnt);
    }

    public void estimate(Mat img, Mat camMatrix, Mat dstMatrix) {


        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Aruco.detectMarkers(img, dict, corners, ids);

        borderColor = new Scalar(255.0, 0.0, 0.0);
        double tx = 0;
        double ty = 0;
        if (ids.size().height > 0) {
            Aruco.drawDetectedMarkers(img, corners, ids, borderColor);

            markerSize = 0.05f;
            rotationMatrix = new Mat();
            rvecs = new Mat();
            tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerSize, camMatrix, dstMatrix, rvecs, tvecs);

            Point3 target_pos = new Point3();


            for (int i = 0; i < rvecs.size().height; i++) {

                Mat rvec = new Mat(1, 3, CvType.CV_64FC1);
                Mat tvec = new Mat(1, 3, CvType.CV_64FC1);
                Mat rot = new Mat();
                rvec.put(0, 0, rvecs.get(i, 0));
                tvec.put(0, 0, tvecs.get(i, 0));
                Calib3d.drawFrameAxes(img, camMatrix, dstMatrix, rvec, tvec, (float) 0.15);

                double _x = 0;
                double _y = 0;
                for (int j = 0; j < corners.size(); j++) {
                    _x = _x + corners.get(i).get(0, j)[0];
                    _y = _y + corners.get(i).get(0, j)[1];
                }
                _x = _x / 4.0;
                _y = _y / 4.0;
                float pos_x = (float) tvec.get(0, 0)[0];
                float pos_y = (float) tvec.get(0, 1)[0];
                float pos_z = (float) tvec.get(0, 2)[0];


                tx = tx + _x;
                ty = ty + _y;

                target_pos.x = target_pos.x + pos_x;
                target_pos.y = target_pos.y + pos_y;
                target_pos.z = target_pos.z + pos_z;


                String AR_position = String.format("(%.2f,%.2f,%.2f)m", pos_x, pos_y, pos_z);
//                Imgproc.putText(img, AR_position, new Point(_x, _y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(14, 201, 255), 1);

//				convert rvec to rotation matrix
                Calib3d.Rodrigues(rvec, rot);

                quaternion orientation = new quaternion();

                orientation.fromRotationMatrix(rot);
                orientation.toEuler();
//				System.out.printf("orien : %.3f %.3f %.3f %.3f\n",orientation.x,orientation.y,orientation.z,orientation.w);
//				System.out.println(ids.get(i,0)[0]);
//				System.out.println(AR_position);
            }
            target_pos.x = target_pos.x / 4.0;
            target_pos.y = target_pos.y / 4.0;
            target_pos.z = target_pos.z / 4.0;

            tx = tx / 4.0;
            ty = ty / 4.0;

            Imgproc.circle(img, new Point(tx, ty), 5, new Scalar(0, 255, 0), -1);

            String target_position = String.format("(%.2f,%.2f,%.2f)m", target_pos.x, target_pos.y, target_pos.z);

            MatOfPoint3f target3D = new MatOfPoint3f();
            target3D.fromArray(target_pos);
            MatOfPoint2f targetImagePlane = new MatOfPoint2f();
            Mat _rvec = new Mat(1, 3, CvType.CV_64FC1);
            Mat _tvec = new Mat(1, 3, CvType.CV_64FC1);
            double[] r = new double[]{0.0f, 0.0f, 0.0f};
            double[] t = new double[]{0.0f, 0.0f, 0.0f};
            _rvec.put(0, 0, r);
            _tvec.put(0, 0, t);

            Calib3d.projectPoints(target3D, _rvec, _tvec, camMatrix, new MatOfDouble(dstMatrix), targetImagePlane);

            //center of target from 3D-world coordinate
//			Point center= new Point(targetImagePlane.get(0, 0)[0],targetImagePlane.get(0, 0)[1]);


            // center of target from image position
//			Point center= new Point(tx,ty);

//			Imgproc.putText(img, target_position, new Point(100,100), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,new Scalar(14,201, 255) , 1);


//			//just for visualize crosshair
//			Point v_start = new Point(center.x, center.y+20);
//		    Point v_end = new Point(center.x, center.y-20);
//
//		    Point h_start = new Point(center.x+20, center.y);
//		    Point h_end = new Point(center.x-20, center.y);
//
//		    Scalar line_color = new Scalar(0, 255,0);
//		    int thickness = 2;
//		    Imgproc.circle(img, center, 8, line_color, 1);
//		    Imgproc.circle(img, center, 16, line_color, 1);
//		    Imgproc.line(img, v_start, v_end, line_color, thickness);
//		    Imgproc.line(img, h_start, h_end, line_color, thickness);
//		    Imgproc.circle(img, center, 4, new Scalar(0,0,255), -1);
//
            //end crosshair
        }

    }
}