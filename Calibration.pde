import gab.opencv.*;
import KinectPV2.*;
KinectPV2 kinect;

//Common Veriables

Circle[] calb_pts;                   //Calibration circles object data(center,diameter)
Point dsp , size ;                   //Display size and Depth img size
Point[] raw_position_in_boundaries;  //Location points of depth pixels via plane and boundaries
float [] mapDCT ;                    //Data of depth pixels location on color frame
PVector[] calb_centers;              //center,corner,right,down
PVector[] raw_cloud;                 //Location vectors of depth pixels via kinects orjin
PVector[] raw_cloud_new;             //Location vectors of depth pixels via projectors orjin
PVector[] boundaries;                //orjin,orjin to right,orjin to down
int [] colorRaw ;                    //Data of pixels color
int [] rawData;                      //Depth data of pixels
int [] depthZero;                    //0 array with size of depth data
int [] frame;                        //Depth frame without background
int [] rawData_bg;                   //Depth pixels background
boolean[] diff_pixel;                //Boolean array for checks there is a difference or not
PVector plane;                       //Normal Vector of the plane that the projector project
PImage depthToColorImg;              
PImage img_depth;
PImage img_color;
PImage color_bg;
int mode =0;
int timer=0;
boolean calibration = true;


//Project Veriables


void setup()
{
  fullScreen();
  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);
  kinect.enableColorImg(true);
  kinect.enablePointCloud(true);
  kinect.init();
  
  depthToColorImg = createImage(960, 540, PImage.RGB);
  depthZero    = new int[960*540];

  //SET THE ARRAY TO 0s
  for (int i = 0; i < 960*540; i++) 
  {
    depthZero[i]=0;
  }

  dsp = new Point(displayWidth,displayHeight);
  size = new Point(kinect.getDepthImage().width,kinect.getDepthImage().height);
  calb_centers = new PVector[4];
  calb_pts = new Circle[4];

}

void draw()
{
  background(255);
  img_color = kinect.getColorImage();
  img_depth = kinect.getDepthImage();
  rawData = kinect.getRawDepthData();
  
  if(calibration)
  {
    mapDCT = kinect.getMapDepthToColor();      
    colorRaw = kinect.getRawColor();

    //PreCalibration
    
    if(mode==0)
    {
      
      //clean de pixels
      PApplet.arrayCopy(depthZero, depthToColorImg.pixels);
    
      int count = 0;
      depthToColorImg.loadPixels();
      for (int i = 0; i < KinectPV2.WIDTHDepth; i++) 
      {
        for (int j = 0; j < KinectPV2.HEIGHTDepth; j++) 
        { 
          float valX = mapDCT[count * 2 + 0];
          float valY = mapDCT[count * 2 + 1];
          int valXDepth = (int)((valX/1920.0) * 960.0);
          int valYDepth = (int)((valY/1080.0) * 540.0);    
          int  valXColor = (int)(valX);
          int  valYColor = (int)(valY);
    
          if ( valXDepth >= 0 && valXDepth < 960 && valYDepth >= 0 && valYDepth < 540 &&
            valXColor >= 0 && valXColor < 1920 && valYColor >= 0 && valYColor < 1080) 
          {
            color colorPixel = colorRaw[valYColor * 1920 + valXColor];
            depthToColorImg.pixels[valYDepth * 960 + valXDepth] = colorPixel;
          }
          count++;
        }
      }
      depthToColorImg.updatePixels();    
      image(depthToColorImg, 480, 270);
      fill(0);
      textSize(70);
      text("Make sure the device covers the image then press any key",100,900);    
    }
    
    //Calibration
    
    else
    {      
      if(mode == 1)
      {
        timer++;
        if(timer>100)
        {
          if(color_bg==null)
          {
            color_bg = img_color;
            rawData_bg = rawData;
          }
          else
          {
            color_bg = GetAvaragePImg(color_bg,img_color);
            rawData_bg = GetAvarageOffTwoIntArray(rawData_bg,rawData);
            if(timer>200)
            {
              mode=2;
              timer=0;
            }
          }
        }
      }
      
      else if (mode==2)
      {
        fill(0);
        circle(dsp.x/2,dsp.y/2,100);
        timer++;
        if(timer>100)
        {
          PImage img = AbssDiffOfTwoPImage(color_bg,img_color);
          calb_pts[mode-2] = GetPointCircle(img);
          timer=0;
          mode =3;
        }
      }
      
      else if (mode==3)
      {
        fill(0);
        circle(dsp.x/4,dsp.y/4,100);
        timer++;
        if(timer>100)
        {
          PImage img = AbssDiffOfTwoPImage(color_bg,img_color);
          calb_pts[mode-2] = GetPointCircle(img);
          timer=0;
          mode =4;
        }
      }
      
      else if (mode==4)
      {
        fill(0);
        circle(3*dsp.x/4,dsp.y/4,100);
        timer++;
        if(timer>100)
        {
          PImage img = AbssDiffOfTwoPImage(color_bg,img_color);
          calb_pts[mode-2] = GetPointCircle(img);
          timer=0;
          mode =5;
        }
      }
      
      else if (mode==5)
      {
        fill(0);
        circle(dsp.x/4,3*dsp.y/4,100);
        timer++;
        if(timer>100)
        {
          PImage img = AbssDiffOfTwoPImage(color_bg,img_color);
          calb_pts[mode-2] = GetPointCircle(img);
          timer=0;
          mode =6;
        }
      }
      else if (mode==6)
      {
        calb_centers = GetCalbPointCenterOnCloud(calb_pts,mapDCT,rawData,size);
        plane = GetPlanesNormal(calb_centers);
        boundaries = GetPlaneROI(calb_centers);
        calibration =false;
        mode = 7;
      }
    }
  }
  else
  {

    frame = AbssDiffOffTwoIntArray(rawData,rawData_bg);
    diff_pixel = DetectDiffPixels(frame,20);
    raw_cloud = Raw2Cloud(rawData,diff_pixel,size);
    raw_cloud_new = SetVectorsWithNewOrigin(raw_cloud,boundaries[0]);    
    raw_position_in_boundaries = GetPointsLocationInBoundaries(raw_cloud_new,boundaries,plane,dsp,80);
    DrawCircleOnPoints(raw_position_in_boundaries,20,255,0,0);
    
  }

if (keyPressed) 
{
  mode=1;
}
}
