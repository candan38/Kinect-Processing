class Point
{
  float x;
  float y;
  
  Point(float a,float b)
  {
    x = a;
    y = b;
  }
  
  Point()
  {
    x = 0;
    y = 0;
  }
}
//*******************************************************************************************************************************************************************
class Circle
{
  Point center;
  int diameter;
  
  Circle(int x,int y,int r)
  {
    center = new Point(x,y);
    diameter=r;
  }
}
//*******************************************************************************************************************************************************************
static class CameraParams
{
  static float cx = 254.878f;
  static float cy = 205.395f;
  static float fx = 365.456f;
  static float fy = 365.456f;
  static float k1 = 0.0905474;
  static float k2 = -0.26819;
  static float k3 = 0.0950862;
  static float p1 = 0.0;
  static float p2 = 0.0;
}
//*******************************************************************************************************************************************************************
int[] GetAvarageOffTwoIntArray(int[]array1, int[]array2)
{
  int size = array1.length;
  int[] result = new int[size];
  for (int i = 0; i< size; i++)
  {
    result[i] = (int)(array1[i]+array2[i])/2;
  }
  return result;
}
//*******************************************************************************************************************************************************************
int[] AbssDiffOffTwoIntArray (int[] array1, int[] array2)
{
  int size = array1.length;
  int[] result = new int[size];
  for (int i = 0; i< size; i++)
  {
    result[i] = (int)abs(array1[i]-array2[i]);
  }
  return result;
}
//*******************************************************************************************************************************************************************
PImage GetAvaragePImg (PImage img1 , PImage img2)
{
  PImage result = createImage(img1.width,img1.height,RGB);
  result.loadPixels();
  img1.loadPixels();
  img2.loadPixels();
  for(int i =0 ; i<img1.width*img1.height;i++)
  {
    float red =(red(img1.pixels[i])+red(img2.pixels[i]))/2;
    float green =(green(img1.pixels[i])+green(img2.pixels[i]))/2;
    float blue =(blue(img1.pixels[i])+blue(img2.pixels[i]))/2;
    color c = color (red,green,blue);
    result.pixels[i] = c; 
  }
  result.updatePixels();
  img1.updatePixels();
  img2.updatePixels();
  return result;  
}
//*******************************************************************************************************************************************************************
PImage AbssDiffOfTwoPImage(PImage img1 , PImage img2)
{
  PImage result = createImage(img1.width,img1.height,RGB);
  result.loadPixels();
  img1.loadPixels();
  img2.loadPixels();
  for(int i =0 ; i<img1.width*img1.height;i++)
  {
    float red =abs(red(img1.pixels[i])-red(img2.pixels[i]));
    float green =abs(green(img1.pixels[i])-green(img2.pixels[i]));
    float blue =abs(blue(img1.pixels[i])-blue(img2.pixels[i]));
    color c = color (red,green,blue);
    result.pixels[i] = c; 
  }
  result.updatePixels();
  img1.updatePixels();
  img2.updatePixels();
  return result;
}
//*******************************************************************************************************************************************************************
void DrawCircleOnPoints(Point[] pts, int dia, int r, int g, int b)
{
  fill(r, g, b);
  for (Point i : pts)
    circle(i.x, i.y, dia);
}
//*******************************************************************************************************************************************************************
Circle GetPointCircle (PImage img)
{
  Circle result = new Circle(0,0,0);
  ArrayList<Contour> contours = new ArrayList<Contour>();
  OpenCV opencv = new OpenCV(this,img);   
  opencv.threshold(20);
  contours=opencv.findContours();
  Contour contour;
  int max =0;
  if(contours.size()>0)
  {
    contour = contours.get(0);
    for(Contour k : contours)
    {     
      if(k.numPoints()>max)
      {
        max = k.numPoints();
        contour = k;
      }
    }
    var bb = contour.getBoundingBox();
    result.center.x = bb.x+bb.width/2;
    result.center.y = bb.y+bb.height/2;
    result.diameter = (int)((bb.width+bb.height)/2);
  }
  return result;
}
//*******************************************************************************************************************************************************************
PVector[] GetCalbPointCenterOnCloud(Circle[] calb_pts , float[] mapDCT ,int[] rawData, Point size)
{  
  ArrayList<Point> mid = new ArrayList<Point>();
  ArrayList<Point> corner = new ArrayList<Point>();
  ArrayList<Point> right = new ArrayList<Point>();
  ArrayList<Point> down = new ArrayList<Point>();
  
  for (int y = 0; y < size.y; y++) 
  {
    for (int x = 0; x < size.x; x++) 
    {
      float valXColor = mapDCT[(int)(x+y*size.x) * 2 + 0];
      float valYColor = mapDCT[(int)(x+y*size.x) * 2 + 1];

      if (valXColor >= 0 && valXColor < 1920 && valYColor >= 0 && valYColor < 1080) 
      {        
        Point pixel = new Point(valXColor,valYColor);        
        Point depth_pixel = new Point(x,y);
        
        if(DistPoint2Point(pixel,calb_pts[0].center)<=calb_pts[0].diameter)
        mid.add(depth_pixel);     
        else if(DistPoint2Point(pixel,calb_pts[1].center)<=calb_pts[1].diameter)
        corner.add(depth_pixel);
        else if(DistPoint2Point(pixel,calb_pts[2].center)<=calb_pts[2].diameter)
        right.add(depth_pixel);
        else if(DistPoint2Point(pixel,calb_pts[3].center)<=calb_pts[3].diameter)
        down.add(depth_pixel);        
      }
    }    
  }
  
  PVector[] result = new PVector[4];
  result[0]=GetPointCenterOnCloud(mid,rawData,size);
  result[1]=GetPointCenterOnCloud(corner,rawData,size);
  result[2]=GetPointCenterOnCloud(right,rawData,size);
  result[3]=GetPointCenterOnCloud(down,rawData,size);
  return result;  
}
//*******************************************************************************************************************************************************************
PVector GetPointCenterOnCloud (ArrayList<Point> pts , int[] rawData , Point size)
{
 float xtotal =0 , ytotal=0 ,ztotal =0;
 int counter =0;
 
 ArrayList<PVector> points = new ArrayList<PVector>();
 for(Point i : pts)
 { 
   PVector v =depthToPointCloudPos((int)i.x,(int)i.y,rawData[(int)i.x+(int)(i.y)*(int)size.x]);   
   points.add(v);   
 }
 
 for(PVector i : points)
 {
   xtotal += i.x;
   ytotal += i.y;
   ztotal += i.z;
   counter++;
 }
 PVector result = new PVector(xtotal/counter,ytotal/counter,ztotal/counter); 
 return result;
}
//*******************************************************************************************************************************************************************
PVector depthToPointCloudPos(int x, int y, float depthValue)
{
  PVector point = new PVector();
  point.z = (depthValue);
  point.x = (x - CameraParams.cx) * point.z / CameraParams.fx;
  point.y = (y - CameraParams.cy) * point.z / CameraParams.fy;
  return point;
}
//*******************************************************************************************************************************************************************
PVector GetPlanesNormal (PVector[] vectors)
{
  PVector v1= GetDiffOf2Vector(vectors[2],vectors[1]);
  PVector v2= GetDiffOf2Vector(vectors[3],vectors[1]);
  PVector v1_copy= v1.copy();
  PVector v2_copy= v2.copy();
  PVector v3 = v1_copy.cross(v2_copy);
  return v3;
}
//*******************************************************************************************************************************************************************
PVector[] GetPlaneROI(PVector[] vectors)
{
  PVector[] result = new PVector[3];
  result[0] = GetDiffOf2Vector(GetVectorTimesNumber(vectors[1],2),vectors[0]);
  result[1] = GetDiffOf2Vector(GetDiffOf2Vector(GetVectorTimesNumber(vectors[2],2),vectors[0]),result[0]);
  result[2] = GetDiffOf2Vector(GetDiffOf2Vector(GetVectorTimesNumber(vectors[3],2),vectors[0]),result[0]);
  return result;
}
//*******************************************************************************************************************************************************************
Point[] GetPointsLocationInBoundaries(PVector[] pts, PVector[] boundaries, PVector plane, Point size ,int dist)
{  
  ArrayList<Point> pre_result = new ArrayList<Point>();
  PVector right = boundaries[1];
  PVector down = boundaries[2];
  float right_size = GetSizeOfVector(right);
  float down_size = GetSizeOfVector(down);

  for (PVector i : pts)
  {
    if (InPlaneRange(i, plane, dist))
    {      
      float xx = (ScalerProduct(i, right)/pow(right_size,2));
      float yy = (ScalerProduct(i, down)/pow(down_size,2));
      if (xx>=0 && xx <=1 && yy>=0 && yy<=1)
      {        
        Point a = new Point();
        a.x = (int)(xx*size.x);
        a.y = (int)(yy*size.y);
        pre_result.add(a);
      }
    }
  }
  
  Point[] result = new Point[pre_result.size()];
  for (int i = 0; i<pre_result.size(); i++)
  {
    result[i] = pre_result.get(i);
  }
  return result;
}
//*******************************************************************************************************************************************************************
boolean InPlaneRange(PVector v, PVector plane, int dist)
{
  float dst = ScalerProduct(v, plane)/GetSizeOfVector(plane);  
  if (abs(dst) <dist)
    return true;
  else return false;
}
//*******************************************************************************************************************************************************************
boolean[] DetectDiffPixels(int[]data, int dist)
{
  boolean[] result = new boolean[data.length];
  for (int i =0; i<data.length; i++)
  {
    if (data[i]<dist)
      result[i]=false;
    else if (data[i]>= dist)
      result[i]=true;
  }
  return result;
}
//*******************************************************************************************************************************************************************
PVector[] Raw2Cloud(int[]raw, boolean[]diff_pixel,Point size )
{
  int counter =0;
  for(boolean i : diff_pixel)
  {
    if(i==true)
    counter++;
  }
  PVector[] result = new PVector[counter];
  int index =0;
  for (int i =0; i<raw.length; i++)
  {
    if (diff_pixel[i])
    {
      int x = i%(int)size.x;
      int y = i/(int)size.x;
      int z = raw[i];
      PVector v = depthToPointCloudPos(x, y, z);
      result[index] = v;
      index++;
    }
  }
  return result;
}
//*******************************************************************************************************************************************************************
PVector[] SetVectorsWithNewOrigin(PVector[] vectors, PVector origin)
{
  PVector[] result = new PVector[vectors.length];
  int index =0;
  for (PVector i : vectors)
  {
    PVector v = GetDiffOf2Vector(i, origin);
    result[index] = v;
    index++;
  }
  return result;
}
//*******************************************************************************************************************************************************************
float ScalerProduct(PVector v1, PVector v2)
{
  float result = (v1.x*v2.x)+(v1.y*v2.y)+(v1.z*v2.z);
  return result;
}
//*******************************************************************************************************************************************************************
float ScalerProduct2D(PVector v1, PVector v2)
{
  float result = (v1.x*v2.x)+(v1.y*v2.y);
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetDiffOf2Vector (PVector v1, PVector v2)
{
  PVector result = new PVector();
  result.x= v1.x-v2.x;
  result.y= v1.y-v2.y;
  result.z= v1.z-v2.z;
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetDiffOf2Vector2D (PVector v1, PVector v2)
{
  PVector result = new PVector();
  result.x= v1.x-v2.x;
  result.y= v1.y-v2.y;
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetVectorTimesNumber(PVector v , float x)
{
  PVector result = new PVector(v.x*x,v.y*x,v.z*x);
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetVectorTimesNumber2D(PVector v , float x)
{
  PVector result = new PVector(v.x*x,v.y*x);
  return result;
}
//*******************************************************************************************************************************************************************
float GetSizeOfVector(PVector v)
{
  float result = sqrt(pow(v.x, 2)+pow(v.y, 2)+pow(v.z, 2));
  return result;
}
//*******************************************************************************************************************************************************************
float GetSizeOfVector2D(PVector v)
{
  float result = sqrt(pow(v.x, 2)+pow(v.y, 2));
  return result;
}
//*******************************************************************************************************************************************************************
float CosOfAngleBetweenTwoVectors(PVector v1, PVector v2)
{
  float result = ScalerProduct(v1, v2)/(GetSizeOfVector(v1)*GetSizeOfVector(v2));
  return result;
}
//*******************************************************************************************************************************************************************
float CosOfAngleBetweenTwoVectors2D(PVector v1, PVector v2)
{
  float result = ScalerProduct2D(v1, v2)/(GetSizeOfVector2D(v1)*GetSizeOfVector2D(v2));
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetUnitVector(PVector v)
{
  PVector result = new PVector(v.x/GetSizeOfVector(v),v.y/GetSizeOfVector(v),v.z/GetSizeOfVector(v));
  return result ;
}
//*******************************************************************************************************************************************************************
PVector GetUnitVector2D(PVector v)
{
  PVector result = new PVector(v.x/GetSizeOfVector2D(v),v.y/GetSizeOfVector2D(v));
  return result ;
}
//*******************************************************************************************************************************************************************
PVector GetProjectionVector(PVector v1, PVector v2)
{
  PVector v2_unit = GetUnitVector(v2);
  float size = GetSizeOfVector(v1);
  float cos = CosOfAngleBetweenTwoVectors(v1, v2);
  PVector result = new PVector(size*cos*v2_unit.x, size*cos*v2_unit.y,size*cos*v2_unit.z);
  return result;
}
//*******************************************************************************************************************************************************************
PVector GetProjectionVector2D(PVector v1, PVector v2)
{
  PVector v2_unit = GetUnitVector2D(v2);
  float size = GetSizeOfVector2D(v1);
  float cos = CosOfAngleBetweenTwoVectors2D(v1, v2);
  PVector result = new PVector(size*cos*v2_unit.x, size*cos*v2_unit.y);
  return result;
}
//*******************************************************************************************************************************************************************
int DistPoint2Point (Point a, Point b)
{
  int h = (int)sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2));
  return h ;
}
