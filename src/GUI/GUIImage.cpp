#include "GUIImage.h"
#include "OgreRectangle2D.h"
#include "OgreHardwarePixelBuffer.h"
using namespace OgreBites;
using namespace Ogre;

GUIImage::GUIImage()
{
    // RenderRectangle = new Ogre::Rectangle2D(true);
}

GUIImage::~GUIImage()
{
    // delete RenderRectangle;
}

void GUIImage::FillTexture(cv::Mat& img)
{
    // Get the pixel buffer
    HardwarePixelBufferSharedPtr pixelBuffer = RectangleTexture->getBuffer();
    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
    const PixelBox &pixelBox = pixelBuffer->getCurrentLock();

    uint8 *pDest = static_cast<uint8 *>(pixelBox.data);

    if (img.isContinuous())
    {
        memcpy(pDest, img.data, img.total() * img.elemSize());
    }
    else
    {
        for (int j = 0; j < img.rows; j++)
        {
            for (int i = 0; i < img.cols; i++)
            {
                *pDest++ = img.at<cv::Vec3b>(j, i)(0); // B
                *pDest++ = img.at<cv::Vec3b>(j, i)(1); // B
                *pDest++ = img.at<cv::Vec3b>(j, i)(2); // B
            }
            pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
        }
    }

    // Unlock the pixel buffer
    pixelBuffer->unlock();
}
