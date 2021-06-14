package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import com.yanzhenjie.zbar.Image;
import com.yanzhenjie.zbar.Config;
import com.yanzhenjie.zbar.BuildConfig;
import com.yanzhenjie.zbar.ImageScanner;
import com.yanzhenjie.zbar.Symbol;
import com.yanzhenjie.zbar.SymbolIterator;
import com.yanzhenjie.zbar.SymbolSet;


public class zbarQR {
	public String qrCodeString = null;

	public void scanQRImage(Mat img){
		int pixel_size = img.width() * img.height()*3;
		Log.d("QR","pixel size " + pixel_size);
		byte[] pixel = new byte[pixel_size];
		img.get(0,0,pixel);
		Log.d("QR","after get");
		Image barcode = new Image(img.width(),img.height(),"Y800");
		Log.d("QR","after barcode");
		barcode.setData(pixel);
		Log.d("QR","after setData");

		ImageScanner reader = new ImageScanner();
		reader.setConfig(Symbol.NONE, Config.ENABLE,0);
		reader.setConfig(Symbol.QRCODE,Config.ENABLE,1);
		int result = reader.scanImage(barcode);
		Log.d("QR","result : " + result);
		if(result != 0){
			SymbolSet symbolSet = reader.getResults();
			for(Symbol symbol : symbolSet){
				qrCodeString = symbol.getData();
			}
		}
	}
	
}
