#pragma once
#include "OgreRectangle2D.h"


class ColoredGradient2D : public Ogre::Rectangle2D
{
public:
    ColoredGradient2D(bool includeTextureCoordinates = false);
    ~ColoredGradient2D(){};

    //sets the colors at the four edges
    void setColors(const Ogre::ColourValue &topLeft, const Ogre::ColourValue & bottomLeft, const Ogre::ColourValue &topRight, const Ogre::ColourValue &bottomRight);
};
