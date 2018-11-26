
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import com.pi4j.io.serial.*;

public class Main {
  public static PrintWriter writer;
  static UsbCamera camera;

  private static void log(String string) {
    writer.println(string);
    writer.close();
  }

  public static void main(String[] args) {
    int streamPort = 1185;
    MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

    camera = setUsbCamera(0, inputStream);
    Runtime.getRuntime().addShutdownHook(new Thread() {
      public void run() {
        if (camera != null) {
          camera.free();
        }
      }
    });
    // Loads our OpenCV library. This MUST be included
    System.loadLibrary("opencv_java310");
    try {
      writer = new PrintWriter("/home/pi/log.txt", "UTF-8");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (UnsupportedEncodingException e) {
      e.printStackTrace();
    }

    // log("this file has been written by team 6644 for some wierd purposes");
    String PI_ADDRESS = "10.66.44.41";
    int PORT = 1185;
    NetworkTable.setClientMode();
    NetworkTable.setTeam(6644);
    NetworkTable.initialize();
    NetworkTable.getTable("CameraPublisher").getSubTable("T. J. Eckleburg").putStringArray("streams",
        new String[] { "mjpeg:http://" + PI_ADDRESS + ":" + PORT + "/stream.mjpg" });

    camera.setResolution(640, 360);
    camera.setFPS(7);

    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);

    // This creates a CvSource to use. This will take in a Mat image that has had
    // OpenCV operations
    // operations

    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
    cvStream.setSource(imageSource);

    Mat image = new Mat();
    double[] hslThresholdHue = { 0.0, 50.98976109215018 };
    double[] hslThresholdSaturation = { 91.72661870503596, 255.0 };
    double[] hslThresholdLuminance = { 80.26079136690647, 255.0 };
    Mat resizedImage = new Mat();
    Mat blurredImage = new Mat();
    Mat HSL_Threshold = new Mat();
    Mat erode = new Mat();
    Mat dilate = new Mat();
    Mat empty = new Mat();
    Scalar Bordervalue = new Scalar(-1);
    double erodeAndDilateIterations = 8.0;
    org.opencv.core.Point anchor = new org.opencv.core.Point(-1, -1);

    // resize variables
    double resizeImageWidth = 320.0;
    double resizeImageHeight = 240.0;
    int resizeImageInterpolation = Imgproc.INTER_CUBIC;

    // blur variables
    double cvMedianblurKsize = 13.0;

    // Blob Variables
    double findBlobsMinArea = 1.0;
    double[] findBlobsCircularity = { 0.0, 1.0 };
    boolean findBlobsDarkBlobs = false;
    MatOfKeyPoint blobsMat = new MatOfKeyPoint();

    while (true) {
      long frameTime = imageSink.grabFrame(image);
      if (frameTime == 0)
        continue;

      resizeImage(image, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizedImage);

      // Step CV_medianBlur:
      cvMedianblur(resizedImage, cvMedianblurKsize, blurredImage);

      // Step HSL_Threshold:
      hslThreshold(blurredImage, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, HSL_Threshold);

      // Step CV_erode:
      cvErode(HSL_Threshold, empty, anchor, erodeAndDilateIterations, Core.BORDER_CONSTANT, Bordervalue, erode);

      // Step CV_dilate:
      cvDilate(erode, empty, anchor, erodeAndDilateIterations, Core.BORDER_CONSTANT, Bordervalue, dilate);

      // Step Find_Blobs:
      findBlobs(dilate, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, blobsMat);

      imageSource.putFrame(HSL_Threshold);
    }

  }

  /**
   * Scales and image to an exact size.
   * 
   * @param input         The image on which to perform the Resize.
   * @param width         The width of the output in pixels.
   * @param height        The height of the output in pixels.
   * @param interpolation The type of interpolation.
   * @param output        The image in which to store the output.
   */
  private static void resizeImage(Mat input, double width, double height, int interpolation, Mat output) {
    Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
  }

  /**
   * Performs a median blur on the image.
   * 
   * @param src   image to blur.
   * @param kSize size of blur.
   * @param dst   output of blur.
   */
  private static void cvMedianblur(Mat src, double kSize, Mat dst) {
    Imgproc.medianBlur(src, dst, (int) kSize);
  }

  /**
   * Segment an image based on hue, saturation, and luminance ranges.
   *
   * @param input  The image on which to perform the HSL threshold.
   * @param hue    The min and max hue
   * @param sat    The min and max saturation
   * @param lum    The min and max luminance
   * @param output The image in which to store the output.
   */
  private static void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum, Mat out) {
    Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
    Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]), new Scalar(hue[1], lum[1], sat[1]), out);
  }

  /**
   * Expands area of lower value in an image.
   * 
   * @param src         the Image to erode.
   * @param kernel      the kernel for erosion.
   * @param anchor      the center of the kernel.
   * @param iterations  the number of times to perform the erosion.
   * @param borderType  pixel extrapolation method.
   * @param borderValue value to be used for a constant border.
   * @param dst         Output Image.
   */
  private static void cvErode(Mat src, Mat kernel, org.opencv.core.Point anchor, double iterations, int borderType,
      Scalar borderValue, Mat dst) {
    if (kernel == null) {
      kernel = new Mat();
    }
    if (anchor == null) {
      anchor = new Point(-1, -1);
    }
    if (borderValue == null) {
      borderValue = new Scalar(-1);
    }
    Imgproc.erode(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
  }

  /**
   * Expands area of higher value in an image.
   * 
   * @param src         the Image to dilate.
   * @param kernel      the kernel for dilation.
   * @param anchor      the center of the kernel.
   * @param iterations  the number of times to perform the dilation.
   * @param borderType  pixel extrapolation method.
   * @param borderValue value to be used for a constant border.
   * @param dst         Output Image.
   */
  private static void cvDilate(Mat src, Mat kernel, Point anchor, double iterations, int borderType, Scalar borderValue,
      Mat dst) {
    if (kernel == null) {
      kernel = new Mat();
    }
    if (anchor == null) {
      anchor = new Point(-1, -1);
    }
    if (borderValue == null) {
      borderValue = new Scalar(-1);
    }
    Imgproc.dilate(src, dst, kernel, anchor, (int) iterations, borderType, borderValue);
  }

  /**
   * Filter out an area of an image using a binary mask.
   * 
   * @param input  The image on which the mask filters.
   * @param mask   The binary image that is used to filter.
   * @param output The image in which to store the output.
   */

  /**
   * Detects groups of pixels in an image.
   * 
   * @param input       The image on which to perform the find blobs.
   * @param minArea     The minimum size of a blob that will be found
   * @param circularity The minimum and maximum circularity of blobs that will be
   *                    found
   * @param darkBlobs   The boolean that determines if light or dark blobs are
   *                    found.
   * @param blobList    The output where the MatOfKeyPoint is stored.
   */
  private static void findBlobs(Mat input, double minArea, double[] circularity, Boolean darkBlobs,
      MatOfKeyPoint blobList) {
    FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
    blobDet.detect(input, blobList);
  }

  private static HttpCamera setHttpCamera(String cameraName, MjpegServer server) {
    NetworkTable publishingTable = NetworkTable.getTable("CameraPublisher");
    while (true) {
      try {
        if (publishingTable.getSubTables().size() > 0) {
          break;
        }
        Thread.sleep(500);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    HttpCamera camera = null;
    if (!publishingTable.containsSubTable(cameraName)) {
      return null;
    }
    ITable cameraTable = publishingTable.getSubTable(cameraName);
    String[] urls = cameraTable.getStringArray("streams", null);
    if (urls == null) {
      return null;
    }
    ArrayList<String> fixedUrls = new ArrayList<String>();
    for (String url : urls) {
      if (url.startsWith("mjpg")) {
        fixedUrls.add(url.split(":", 2)[1]);
      }
    }
    camera = new HttpCamera("CoprocessorCamera", fixedUrls.toArray(new String[0]));
    server.setSource(camera);
    return camera;
  }

  private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
    server.setSource(camera);
    return camera;
  }
}
