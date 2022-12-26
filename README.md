# Kinect-Processing
Automatic calibration of the Kinect device on the image output from the Projector



Thanks to this algorithm, we can find the plane on which the projection device is reflected and the boundaries of this plane, and calibrate the kinect device automatically, and we can obtain the location of objects with a certain length range from this plane very precisely on this plane.


STEP 1 --PRE-CALİBRATİON
  
  *When the program runs for the first time, you will see a white screen with the color image coming from the RGB camera of the Kinect device in the center. All you     have to do is to direct your Kinekt device to the image projected by the projector with the help of the image coming from the camera and ensure that the camera         captures the whole image in the frame. Then you can continue the program by pressing any key.
  
  
STEP 2 --CALİBRATİON

  *At this stage, the program will automatically complete the calibration by following the steps below.

  NOTE:Make sure that there is no movement in the field of view of the device and that the ambient light is not changed until this stage is over.
  
  
  1) At this stage , a white image will be projected from the projector . In this phase, which takes an average of 5 seconds, the program will collect data from both        the RGB camera and the depth sensors and will use this data in the next stages to create the background.
  
  2) At this stage, the projector will project 4 different images with a black circle on a white background, respectively. These images in different positions will be        read by the RGB camera
  
  3) At this stage, the positions of these 4 calibration circles read by the RGB camera on the real-size point cloud will be calculated. Then following the steps below        will find the plane and plane boundaries
  
      a) Plane normal vector can be found with the help of 3 points given on the plane. At this stage, we find an average vector for all points inside the 3 circles to            get a more precise result. We find the plane normal vector with the help of about 1000 points at an average distance
      b) Then again, using the positions of the points on these 4 circles on the point cloud, we find the upper left corner of the reflected image, the vector drawn            from the upper left corner to the upper right corner (upper) and the vector drawn from the upper left corner to the lower left corner (side).
      
      ...And its done
      
STEP 3 (RUN)
  
  *Now the calibration is complete and after this stage, the depth sensors will examine the changed pixels using the previously created background. The sensors do not    measure the values stably, there are deviations of a few mm in every frame. We first send the background removed image to a function that takes parameters in mm .
   
   (Function:DetectDiffPixels)
   
   This parameter specifies the minimum value required for the function to accept a change in pixel . The ideal value can be adjusted between 5 and 10 mm depending on    the distance.
   
   Thanks to this step, we have 2 advantages. Firstly, we ignore the small amount of changes coming from the sensor, although there is no change. Secondly , we            determine only the pixels where the change occurs . In this way, we provide a very good efficiency in terms of performance by not processing unnecessary pixels.
   
   
  *We find the positions on the point cloud of the points detected to be change
   (Function:Raw2Cloud)
   
  *We draw vectors to these points in the point cloud from the upper left corner of the plane(origin) boundary we found earlier.
   (Function:SetVectorsWithNewOrigin)
   
  *Finally, our last function uses the plane normal vector to find the distances of these points to the plane. We filter the points with proximity below a certain        distance with the parameter in millimeters that the function takes. Then we obtain the locations of these points on this plane using the plane boundaries . And        until the program is terminated, every frame is processed in this way and gives us the coordinates of these points. You can use these points as you wish in the        application you have developed.
   (Function:GetPointsLocationInBoundaries)
   
   *The program works on an average computer with a performance of about 50 FPS.
   
   
   Note :
         The content will be supported and updated with images. Also , a related video will be shared soon . Your opinions, suggestions and criticisms are important to          me, I would be happy if you share them.
         
         Abdussamet Candan
      
