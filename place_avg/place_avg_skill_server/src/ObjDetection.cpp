#include "ObjDetection.hpp"

float ObjDetec::ox=0;
float ObjDetec::oy=0;
float ObjDetec::oz=0;

void ObjDetec::pointsToPose(geometry_msgs::Point p1,geometry_msgs::Point p2){
    float xp=p1.x;
    float yp=p1.y;
    float zp=p1.z;
    float xc=p2.x;
    float yc=p2.y;
    float zc=p2.z;
    const float pid= 180/M_PI;
    ObjDetec::ox = atan(sqrt(pow((xp-xc),2)+pow((yp-yc),2) )/(zp-zc));
    ObjDetec::oy = atan(sqrt( pow((zp-zc),2)+pow((yp-yc),2))/(xp-xc));
    ObjDetec::oz = atan(sqrt(pow((xp-xc),2)+pow((zp-zc),2))/(yp-yc));
    cout << "Angulo x:" << ObjDetec::ox*pid << endl;
    cout << "Angulo y:" << ObjDetec::oy*pid << endl;
    cout << "Angulo z:" << ObjDetec::oz*pid << endl;
}


geometry_msgs::Point ObjDetec::pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
{
    geometry_msgs::Point p;
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
        
    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;

    return p;
}

Point ObjDetec::objDetec(Kinect cam,string image_path){
   
    if (cam.get_image().data){
            Mat img=cam.get_image();

            //Aplica Orb na imagem
            std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
            cv::Mat descriptors_object, descriptors_scene;
            double ti=(double)getTickCount();
            Ptr<Feature2D> detector= ORB::create(2000, 1.2f, 8, 10 , 0, 2, ORB::HARRIS_SCORE, 10);
            Ptr<Feature2D> extractor= ORB::create() ;
            Mat out,img_scene;
            cvtColor(img, img_scene , cv::COLOR_BGR2GRAY);
            double t=(double)getTickCount();
            detector->detect(img_scene, keypoints_scene);
            extractor->compute(img_scene, keypoints_scene, descriptors_scene);
            t = ((double)getTickCount() - t)/getTickFrequency();
            cout << "Tempo de Processamento: " << t << endl;
            drawKeypoints(img_scene, keypoints_scene, out, Scalar::all(255));
            imshow("Cena",out);

            Mat img_object=imread(image_path, CV_LOAD_IMAGE_COLOR);
            cvtColor(img_object, img_object , cv::COLOR_BGR2GRAY);
            detector->detect(img_object,  keypoints_object);
            extractor->compute(img_object, keypoints_object, descriptors_object);
            Mat out2;
            drawKeypoints(img_object, keypoints_object, out2, Scalar::all(255));
            imshow("Objeto",out2);



            //Compara as features
            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
            vector<cv::DMatch> matches;
            Mat img_matches;
            if(!descriptors_object.empty() && !descriptors_scene.empty()) {
                double t1=(double)getTickCount();
                matcher->match (descriptors_object, descriptors_scene, matches);
                t1 = ((double)getTickCount() - t1)/getTickFrequency();
                cout << "Tempo de Processamento de Matches: " << t1 << endl;

                double max_dist = 0; double min_dist = 100;

                //-- Quick calculation of max and min idstance between keypoints
                for( int i = 0; i < descriptors_object.rows; i++)
                { double dist = matches[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                    if( dist > max_dist ) max_dist = dist;
                }
            
                //-- Draw only good matches (i.e. whose distance is less than 3*min_dist)
                std::vector< cv::DMatch >good_matches;

                for( int i = 0; i < descriptors_object.rows; i++ )

                { if( matches[i].distance < (max_dist/1.6) )
                    { good_matches.push_back( matches[i]); }
                }

                cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, \
                        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                //-- localize the object
                std::vector<Point2f> obj;
                std::vector<Point2f> scene;

                for( size_t i = 0; i < good_matches.size(); i++) {
                    //-- get the keypoints from the good matches
                    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
                }
                if( !obj.empty() && !scene.empty() && good_matches.size() >= 4) {
                    double t2=(double)getTickCount();
                    Mat H = findHomography( obj, scene, RANSAC );
                    t2 = ((double)getTickCount() - t2)/getTickFrequency();
                    cout << "Tempo de Processamento do RANSAC: " << t2 << endl;

                    //-- get the corners from the object to be detected
                    std::vector<Point2f> obj_corners(4);
                    obj_corners[0] = Point(0,0);
                    obj_corners[1] = Point(img_object.cols,0);
                    obj_corners[2] = Point(img_object.cols,img_object.rows);
                    obj_corners[3] = Point(0,img_object.rows);

                    std::vector<cv::Point2f> scene_corners(4);

                    cv::perspectiveTransform( obj_corners, scene_corners, H);

                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )

                    Point p1= scene_corners[0] + Point2f(img_object.cols, 0);
                    Point p2= scene_corners[1] + Point2f(img_object.cols, 0);
                    Point p3= scene_corners[2] + Point2f(img_object.cols, 0);
                    Point p4= scene_corners[3] + Point2f(img_object.cols, 0);

                    line( img_matches, p1, p2, cv::Scalar(0,0,255), 4 );
                    line( img_matches, p2, p3, cv::Scalar(0,255,0), 4 );
                    line( img_matches, p3, p4, cv::Scalar(0,255,0), 4 );
                    line( img_matches, p4, p1, cv::Scalar(0,255,0), 4 );

                    ObjDetec::ox=atan2(p2.y - p1.y, p2.x - p1.x);
                    cout << "Rotacao: " << ObjDetec::ox*180/M_PI << endl;
                    /*
                    Point t,t2;
                    t.x= p1.x - 5;
                    t.y= p1.y;
                    t2.x= p2.x + 5;
                    t2.y= p2.y;

                    auto c1=ObjDetec::pixelTo3DPoint(cam.get_cloud(),p1.x,p1.y);
                    auto c2=ObjDetec::pixelTo3DPoint(cam.get_cloud(),p2.x,p2.y);

                    cout << "C1: " << c1 << endl;
                    cout << "C2: " << c2 << endl;

                    pointsToPose(c2,c1);
                    */

                    auto x=(p1.x+p2.x+p3.x+p4.x)/4 ;
                    auto y=(p1.y+p2.y+p3.y+p4.y)/4;

                    Point center(x,y);

                    circle( img_matches, center, 4, cv::Scalar(255,255,255), -1, 8, 0 );
                    imshow("match result", img_matches );
                    x= x - img_object.cols;
                    center.x=x;
                    ti = ((double)getTickCount() - ti)/getTickFrequency();
                    cout << "Tempo de Processamento Total: " << ti << endl;
                    return center;
                }

            }
    
    }
    Point center_void(0,0);
    return center_void;
}

void ObjDetec::decode(Mat &im, vector<decodedObject>&decodedObjects){

  ImageScanner scanner;
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

  Mat imGray;
  cvtColor(im, imGray,CV_BGR2GRAY);
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
  int n = scanner.scan(image);

  cout << n << endl;

  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;
     
    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;
     
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }
     
    decodedObjects.push_back(obj);
  }
}

Point ObjDetec::display(Mat &im, vector<decodedObject>&decodedObjects, string code, bool show)
{
  Point center(0,0);
  // Loop over all decoded objects
  for(unsigned int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;
     
    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
     
    // Number of points in the convex hull
    int n = hull.size();
     
    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    }
    
    if (decodedObjects[i].data == code){
      auto u= (decodedObjects[i].location[0].x + decodedObjects[i].location[1].x + decodedObjects[i].location[2].x + decodedObjects[i].location[3].x)/4;
      auto v= (decodedObjects[i].location[0].y + decodedObjects[i].location[1].y + decodedObjects[i].location[2].y + decodedObjects[i].location[3].y)/4;
      center.x=u;
      center.y=v;
      putText(im, decodedObjects[i].data , cvPoint(center.x,center.y), 
      FONT_HERSHEY_TRIPLEX, 0.8, cvScalar(0,0,250), 1, CV_AA);
    }
     
  }
  if (show){
    // Display results 
    imshow("QR Results", im);
  }
  return center;
}

Point ObjDetec::qrDetect(Kinect cam,string code,bool show){
    if (cam.get_image().data){
      Mat im = cam.get_image();

      vector<decodedObject> decodedObjects;
      
      decode(im, decodedObjects);
      
      return display(im, decodedObjects, code, show);
    }else{
      Point center(0,0);
      return center;
    }
}

TagTestOptions ObjDetec::parse_options(int argc, char** argv) {
  TagTestOptions opts;
  const char* options_str = "hdtRvxoDS:s:a:m:V:N:brnf:e:";
  int c;
  while ((c = getopt(argc, argv, options_str)) != -1) {
    switch (c) {
      // Reminder: add new options to 'options_str' above and print_usage()!
      case 'd': opts.show_debug_info = true; break;
      case 't': opts.show_timing = true; break;
      case 'R': opts.show_results = true; break;
      case 'v': opts.be_verbose = true; break;
      case 'x': opts.no_images = true; break;
      case 'o': opts.generate_output_files = true; break;
      case 'D': opts.params.segDecimate = true; break;
      case 'S': opts.params.sigma = atof(optarg); break;
      case 's': opts.params.segSigma = atof(optarg); break;
      case 'a': opts.params.thetaThresh = atof(optarg); break;
      case 'm': opts.params.magThresh = atof(optarg); break;
      case 'V': opts.params.adaptiveThresholdValue = atof(optarg); break;
      case 'N': opts.params.adaptiveThresholdRadius = atoi(optarg); break;
      case 'b': opts.params.refineBad = true; break;
      case 'r': opts.params.refineQuads = true; break;
      case 'n': opts.params.newQuadAlgorithm = true; break;
      case 'f': opts.family_str = optarg; break;
      case 'e': opts.error_fraction = atof(optarg); break;
      default:
        fprintf(stderr, "\n");
        exit(1);
    }
  }
  opts.params.adaptiveThresholdRadius += (opts.params.adaptiveThresholdRadius+1) % 2;
  if (opts.be_verbose) {
    opts.show_debug_info = opts.show_timing = opts.show_results = true;
  }
  return opts;
}

Point ObjDetec::aprilDetect(Kinect cam, unsigned int id, bool show){
  const std::string win = "Tag test";
  Point zero(0,0);

  TagTestOptions opts = parse_options(0, 0);

  TagFamily family(opts.family_str);

  if (opts.error_fraction >= 0 && opts.error_fraction < 1) {
    family.setErrorRecoveryFraction(opts.error_fraction);
  }

  TagDetector detector(family, opts.params);
  detector.debug = opts.show_debug_info;
  detector.debugWindowName = opts.generate_output_files ? "" : win;
  if (opts.params.segDecimate && opts.be_verbose) {
    std::cout << "Will decimate for segmentation!\n";
  }

  TagDetectionArray detections;
  Mat src = cam.get_image();
  
  if (src.empty()) { return zero; }

  while (std::max(src.rows, src.cols) > 800) {
    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(0,0), 0.5, 0.5);
    src = tmp;
  }
  cv::Point2d opticalCenter(0.5*src.rows, 0.5*src.cols);

  clock_t start = clock();
  detector.process(src, opticalCenter, detections);
  clock_t end = clock();

  if (opts.show_results) {
    if (opts.show_debug_info) std::cout << "\n";
    std::cout << "Got " << detections.size() << " detections in "
              << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
    for (size_t i=0; i<detections.size(); ++i) {
      const TagDetection& d = detections[i];
      std::cout << " - Detection: id = " << d.id << ", "
                << "code = " << d.code << ", "
                << "position = " << d.cxy << ", "
                << "rotation = " << d.rotation << "\n";
      cv::Point center(d.cxy.x,d.cxy.y);
      cv::circle( src, center, 4, cv::Scalar(0,0,255), -1, 8, 0 );
    }
  }
  if (!opts.no_images) {
    cv::Mat img = family.superimposeDetections(src, detections);
    labelAndWaitForKey(win, "Detected", img, ScaleNone, true);
  }
  
   for (size_t i=0; i<detections.size(); ++i) {
      const TagDetection& d = detections[i];
      std::cout << " - Detection: id = " << d.id << ", "
                << "code = " << d.code << ", "
                << "position = " << d.cxy << ", "
                << "rotation = " << d.rotation << "\n";
      if (d.id == id){
        cv::Point center(d.cxy.x,d.cxy.y);
        cv::circle( src, d.p[0], 4, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle( src, d.p[1], 4, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle( src, d.p[2], 4, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle( src, d.p[3], 4, cv::Scalar(255,0,0), -1, 8, 0 );
        cv::circle( src, center, 4, cv::Scalar(0,0,255), -1, 8, 0 );
        ObjDetec::ox=atan2(d.p[1].y - d.p[0].y, d.p[1].x - d.p[0].x);
        cout << "Rotacao: " << ObjDetec::ox*180/M_PI << endl;
        putText(src, to_string(d.id) , cvPoint(center.x,center.y),FONT_HERSHEY_DUPLEX, 0.8, cvScalar(0,0,250), 1, CV_AA);
        return center;
      }
     
    }
  
  if (opts.show_timing) detector.reportTimers();
  return zero;
}

void ObjDetec::multAprilDetect(Kinect cam, unsigned int id[], Point output[], int size, bool show){

    const std::string win = "Tag test";
    Point zero(0,0);

    for (int i=0;i < size;i++){
        output[i]=zero;
    }

    TagTestOptions opts = parse_options(0, 0);

    TagFamily family(opts.family_str);

    if (opts.error_fraction >= 0 && opts.error_fraction < 1) {
      family.setErrorRecoveryFraction(opts.error_fraction);
    }

    TagDetector detector(family, opts.params);
    detector.debug = opts.show_debug_info;
    detector.debugWindowName = opts.generate_output_files ? "" : win;
    if (opts.params.segDecimate && opts.be_verbose) {
      std::cout << "Will decimate for segmentation!\n";
    }

    TagDetectionArray detections;
    Mat src = cam.get_image();

    if (!src.empty()) {

        while (std::max(src.rows, src.cols) > 800) {
          cv::Mat tmp;
          cv::resize(src, tmp, cv::Size(0,0), 0.5, 0.5);
          src = tmp;
        }
        cv::Point2d opticalCenter(0.5*src.rows, 0.5*src.cols);

        clock_t start = clock();
        detector.process(src, opticalCenter, detections);
        clock_t end = clock();

        if (opts.show_results) {
          if (opts.show_debug_info) std::cout << "\n";
          std::cout << "Got " << detections.size() << " detections in "
                    << double(end-start)/CLOCKS_PER_SEC << " seconds.\n";
          for (size_t i=0; i<detections.size(); ++i) {
            const TagDetection& d = detections[i];
            std::cout << " - Detection: id = " << d.id << ", "
                      << "code = " << d.code << ", "
                      << "position = " << d.cxy << ", "
                      << "rotation = " << d.rotation << "\n";
            cv::Point center(d.cxy.x,d.cxy.y);
            cv::circle( src, center, 4, cv::Scalar(0,0,255), -1, 8, 0 );
          }
        }
        if (!opts.no_images) {
          cv::Mat img = family.superimposeDetections(src, detections);
          labelAndWaitForKey(win, "Detected", img, ScaleNone, true);
        }

         for (size_t i=0; i<detections.size(); ++i) {
            const TagDetection& d = detections[i];
            std::cout << " - Detection: id = " << d.id << ", "
                      << "code = " << d.code << ", "
                      << "position = " << d.cxy << ", "
                      << "rotation = " << d.rotation << "\n";
            for (int j=0;j < size;j++){
                if (d.id == id[j]){
                  cv::Point center(d.cxy.x,d.cxy.y);
                  cv::circle( src, d.p[0], 4, cv::Scalar(255,0,0), -1, 8, 0 );
                  cv::circle( src, d.p[1], 4, cv::Scalar(255,0,0), -1, 8, 0 );
                  cv::circle( src, d.p[2], 4, cv::Scalar(255,0,0), -1, 8, 0 );
                  cv::circle( src, d.p[3], 4, cv::Scalar(255,0,0), -1, 8, 0 );
                  cv::circle( src, center, 4, cv::Scalar(0,0,255), -1, 8, 0 );
                  putText(src, to_string(d.id) , cvPoint(center.x,center.y),FONT_HERSHEY_DUPLEX, 0.8, cvScalar(0,0,250), 1, CV_AA);
                  output[j]=center;
                  //return center;
                }
            }


          }
    }
}
