#include "ColoredGradient2D.h"
#include "OgreHardwareBufferManager.h"

#define COLOUR_BINDING 3


ColoredGradient2D::ColoredGradient2D(bool includeTextureCoordinates /*= false*/):Ogre::Rectangle2D(includeTextureCoordinates)
{
    Ogre::VertexDeclaration* dec1 = mRenderOp.vertexData->vertexDeclaration;
    dec1->addElement(COLOUR_BINDING,0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    Ogre::VertexBufferBinding* bind = mRenderOp.vertexData->vertexBufferBinding;

    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(dec1->getVertexSize(COLOUR_BINDING),mRenderOp.vertexData->vertexCount,Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    //Bind buffer
    bind->setBinding(COLOUR_BINDING, vbuf);
}


void ColoredGradient2D::setColors(const Ogre::ColourValue &topLeft, const Ogre::ColourValue & bottomLeft, const Ogre::ColourValue &topRight, const Ogre::ColourValue &bottomRight)
{
    Ogre::HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(COLOUR_BINDING);
    unsigned int* pUnit32 = static_cast<unsigned int*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    const Ogre::VertexElementType srcType = Ogre::VertexElement::getBestColourVertexElementType();

    *pUnit32++ = Ogre::VertexElement::convertColourValue(topLeft, srcType);
    *pUnit32++ = Ogre::VertexElement::convertColourValue(bottomLeft, srcType);
    *pUnit32++ = Ogre::VertexElement::convertColourValue(topRight, srcType);
    *pUnit32++ = Ogre::VertexElement::convertColourValue(bottomRight, srcType);

    vbuf->unlock();
}
