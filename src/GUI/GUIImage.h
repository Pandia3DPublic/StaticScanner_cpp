#pragma once

class GUIImage
{
private:
    /* data */
public:
    GUIImage();
    ~GUIImage();

    Ogre::Rectangle2D * RenderRectangle;
    Ogre::TexturePtr RectangleTexture;
    Ogre::MaterialPtr RectangleMaterial;

    void FillTexture(cv::Mat& img);
};
