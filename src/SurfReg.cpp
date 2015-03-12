#include <iostream>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include "ThreadMutexObject.h"

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
ThreadMutexObject<bool> done(false);
ThreadMutexObject<int> count(0);
ThreadMutexObject<int> iteration(0);

float getScore(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input)
{
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(mCloud);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    double totalSum = 0;

    count.assignValue(0);

    for(size_t i = 0; i < input->size(); i++)
    {
        tree->nearestKSearch(input->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        totalSum += sqrt(pointNKNSquaredDistance.at(0));
        count++;
    }

    return totalSum / (double)input->size();
}

void computeAlignment()
{
    float value = getScore(rCloud);

    if(value < 0.05)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned (new pcl::PointCloud <pcl::PointXYZRGBNormal>);

        icp.setInputSource(rCloud);
        icp.setInputTarget(mCloud);
        icp.setMaximumIterations(1);

        for(int i = 0; i < 10; i++)
        {
            icp.align(*aligned, icp.getFinalTransformation());
            iteration++;
        }

        value = std::min(getScore(aligned), value);
    }

    std::cout << value << std::endl;

    done.assignValue(true);
}

int main(int argc, char ** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::string reconstructionFile;
    pcl::console::parse_argument(argc, argv, "-r", reconstructionFile);

    std::string modelFile;
    pcl::console::parse_argument(argc, argv, "-m", modelFile);

    if(reconstructionFile.length() == 0 || modelFile.length() == 0)
    {
        std::cout << "Please provide input files with -r and -m" << std::endl;
        exit(1);
    }

    pcl::PCLPointCloud2 rPoints;

    pcl::toPCLPointCloud2(*rCloud, rPoints);

    for(int i = 0; i < rPoints.fields.size(); i++)
    {
        if(rPoints.fields.at(i).name.compare("curvature") == 0)
        {
            rPoints.fields.at(i).name = "radius";
        }
    }

    pcl::io::loadPLYFile(reconstructionFile, rPoints);

    pcl::fromPCLPointCloud2(rPoints, *rCloud);

    pcl::PCLPointCloud2 mPoints;

    pcl::toPCLPointCloud2(*mCloud, mPoints);

    for(int i = 0; i < mPoints.fields.size(); i++)
    {
        if(mPoints.fields.at(i).name.compare("curvature") == 0)
        {
            mPoints.fields.erase(mPoints.fields.begin() + i);
            i--;
        }
    }

    pcl::io::loadPLYFile(modelFile, mPoints);

    pcl::fromPCLPointCloud2(mPoints, *mCloud);

    int trajectory = 0;
    pcl::console::parse_argument(argc, argv, "-t", trajectory);

    Eigen::Matrix4f trans;

    if(trajectory == 0)
    {
        trans << 0.999759, -0.000287637, 0.0219655, -1.36022,
                 0.000160294, 0.999983, 0.00579897, 1.48382,
                 0.0219668, 0.00579404, -0.999742, 1.44256,
                 0, 0, 0, 1;
    }
    else if(trajectory == 1)
    {
        trans << 0.99975, -0.00789018, 0.0209474, -0.0976324,
                 0.00789931, 0.999969, -0.000353282, 1.27618,
                 0.0209439, -0.000518671, -0.999781, 0.0983734,
                 0, 0, 0, 1;
    }
    else if(trajectory == 2)
    {
        trans << 0.999822, 0.0034419, 0.0185526, -0.786316,
                 -0.00350915, 0.999987, 0.00359374, 1.28433,
                 0.01854, 0.00365819, -0.999821, 1.45583,
                 0, 0, 0, 1;
    }
    else if(trajectory == 3)
    {
        trans << 0.999778, -0.000715914, 0.0210893, -1.13311,
                 0.000583688, 0.99998, 0.0062754, 1.26825,
                 0.0210934, 0.0062617, -0.999758, 0.901866,
                 0, 0, 0, 1;
    }

    pcl::transformPointCloud(*rCloud, *rCloud, trans);

    pcl::visualization::PCLVisualizer cloudViewer;
    cloudViewer.setBackgroundColor(0, 0, 0);
    cloudViewer.initCameraParameters();
    int argcam = 4;
    char * cameraArgs [] = {"", "-cam", "0.0188428,18.8428/-0.62153,0.55779,-0.0789679/3.62544,4.44186,3.40479/-0.511828,0.811792,-0.281115/0.8575/1920,1033/0,46", "bla"};

    cloudViewer.getCameraParameters(argcam, cameraArgs);
    cloudViewer.updateCamera();
    cloudViewer.setSize(1680, 1050);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> color(rCloud);
    cloudViewer.addPointCloud<pcl::PointXYZRGBNormal>(rCloud, color, "Cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
    cloudViewer.addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
    cloudViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "MCloud");

    boost::thread * computeThread = new boost::thread(computeAlignment);

    while(!done.getValue())
    {
        cloudViewer.spinOnce(330, true);
        cloudViewer.removeShape("text");

        int countVal = count.getValue();

        if(countVal == rCloud->size())
        {
            std::stringstream strs;

            strs << "Aligning... " << iteration.getValue() << "/" << 10;

            cloudViewer.addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");
        }
        else
        {
            std::stringstream strs;

            strs << "Scoring... " << countVal << "/" << rCloud->size();

            cloudViewer.addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");
        }
    }

    computeThread->join();

    delete computeThread;
}
